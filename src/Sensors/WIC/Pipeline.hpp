//***************************************************************************
// Copyright 2007-2023 EvoLogics GmbH                                       *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Universidade do Porto. For licensing   *
// terms, conditions, and further information contact lsts@fe.up.pt.        *
//                                                                          *
// European Union Public Licence - EUPL v.1.1 Usage                         *
// Alternatively, this file may be used under the terms of the EUPL,        *
// Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Michael Purser                                                   *
//***************************************************************************

#ifndef SENSORS_WIC_PIPELINE_HPP_INCLUDED_
#define SENSORS_WIC_PIPELINE_HPP_INCLUDED_

// C++ STL headers.
#include <string>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Library headers.
#include <glib.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// Local headers.
#include "Constants.hpp"
#include "GstWrappers.hpp"

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    enum PipelineState
    {
      NOT_PLAYING,
      PLAYING,
      ERROR,
    };

    struct PipelineSettings
    {
      bool use_hw_encoding;
      bool record_to_file;
      int frame_width;
      int frame_height;
      int src_framerate;
      int stream_framerate;
      std::string udp_destination;
    };

    class Pipeline
    {
    public:
      Pipeline(Tasks::Task* parent):
        m_parent {parent}
      {
      }

      ~Pipeline()
      {
        stop();
        cleanup();
      }

      void
      initialize()
      {
        gst_init(nullptr, nullptr);
      }

      void
      updateSettings(const PipelineSettings& settings)
      {
        if (settings.use_hw_encoding != m_settings.use_hw_encoding)
          m_parent->inf("%s", settings.use_hw_encoding ? "using hardware encoding" : "using software encoding");
        if (settings.record_to_file != m_settings.record_to_file)
          m_parent->inf("%s", settings.record_to_file ? "recording enabled" : "recording disabled");
        if (settings.frame_width != m_settings.frame_width)
          m_parent->inf("set source frame width to: %d px", settings.frame_width);
        if (settings.frame_height != m_settings.frame_height)
          m_parent->inf("set source frame height to: %d px", settings.frame_height);
        if (settings.src_framerate != m_settings.src_framerate)
          m_parent->inf("set source framerate to: %d FPS", settings.src_framerate);
        if (settings.stream_framerate != m_settings.stream_framerate)
          m_parent->inf("set stream framerate to: %d FPS", settings.stream_framerate);
        if (settings.udp_destination != m_settings.udp_destination)
          m_parent->inf("set UDP destination to: %s", settings.udp_destination.c_str());

        m_settings = settings;
      }

      FileSystem::Path
      recordDirectory() const
      {
        return m_record_directory;
      }

      void
      setRecordDirectory(const FileSystem::Path& record_directory)
      {
        m_record_directory = record_directory;
      }

      std::string
      error() const
      {
        return m_error;
      }

      PipelineState
      state() const
      {
        if (!m_error.empty())
          return PipelineState::ERROR;
        if (m_pipeline == nullptr)
          return PipelineState::NOT_PLAYING;
        return (GST_STATE(m_pipeline) == GST_STATE_PLAYING) ? PipelineState::PLAYING : PipelineState::NOT_PLAYING;
      }

      PipelineSettings
      settings() const
      {
        return m_settings;
      }

      bool
      create()
      {
        m_parent->debug("creating pipeline");

        if (!createPipelineAndBus())
          return false;
        if (!createSource())
          return false;
        if (!createTee())
          return false;
        if (!createBins())
          return false;
        if (!addElementsToPipeline())
          return false;
        if (!linkPipeline())
          return false;

        return true;
      }

      void
      cleanup()
      {
        if (m_pipeline != nullptr)
        {
          m_parent->debug("unreferencing pipeline");
          gst_object_unref(m_pipeline);
          m_pipeline = nullptr;
        }
        if (m_bus != nullptr)
        {
          m_parent->debug("unreferencing bus");
          gst_object_unref(m_bus);
          m_bus = nullptr;
        }
      }

      bool
      start()
      {
        m_parent->debug("starting pipeline");

        if (!checkReadyToStart())
          return false;
        if (!setPipelinePlaying())
          return false;

        return true;
      }

      bool
      stop()
      {
        if (m_pipeline == nullptr)
          return true;

        m_parent->debug("stopping pipeline");

        if (!setPipelineStopping())
          return false;

        return true;
      }

      void
      monitorBus()
      {
        while (gst_bus_peek(m_bus) != nullptr)
          treatBusMsg();
      }

      void
      pushData(uint8_t* data)
      {
        if (data == nullptr)
          return;

        GstSample* sample;
        GstBuffer* buffer;
        GstFlowReturn ret;

        constexpr size_t bytes_per_pixel = 3U; // RGB
        const size_t data_size = m_settings.frame_width * m_settings.frame_height * bytes_per_pixel;

        buffer = gst_buffer_new_allocate(nullptr, data_size, nullptr);
        if (buffer == nullptr)
          return;

        size_t num_filled = gst_buffer_fill(buffer, 0, data, data_size);
        if (num_filled != data_size)
          return;

        sample = gst_sample_new(buffer, m_appsrc->sourceCaps(), nullptr, nullptr);
        if (sample == nullptr)
          return;

        ret = gst_app_src_push_sample(GST_APP_SRC_CAST(m_appsrc->element()), sample);
        if (ret != GST_FLOW_OK)
          return;

        gst_sample_unref(sample);
        gst_buffer_unref(buffer);
      }

    private:
      bool
      createPipelineAndBus()
      {
        m_pipeline = gst_pipeline_new("pipeline");
        m_bus = gst_pipeline_get_bus(GST_PIPELINE(m_pipeline));

        if (m_pipeline == nullptr || m_bus == nullptr)
        {
          m_error = "failed to create gstreamer pipeline and/or bus";
          return false;
        }

        return true;
      }

      bool
      createSource()
      {
        m_appsrc = new GstElementWrapper {"source", "appsrc"};
        m_appsrc->setSourceCaps(String::str("video/x-raw, format=RGB, width=%d, height=%d, framerate=%d/1",
                                            m_settings.frame_width,
                                            m_settings.frame_height,
                                            m_settings.src_framerate));
        m_appsrc->setProperty("stream-type", 0);
        m_appsrc->setProperty("format", GST_FORMAT_TIME);
        m_appsrc->setProperty("is-live", true);
        m_appsrc->setProperty("do-timestamp", true);

        if (!m_appsrc->valid())
        {
          m_error = "failed to create appsrc element";
          return false;
        }
        return true;
      }

      bool
      createTee()
      {
        m_tee = new GstElementWrapper {"tee", "tee"};

        if (!m_tee->valid())
        {
          m_error = "failed to create tee element";
          return false;
        }

        return true;
      }

      void
      selectBins()
      {
        if (m_settings.use_hw_encoding)
        {
          m_source_bin = new SourceBinHWEnc(m_settings.frame_width, m_settings.frame_height, m_settings.src_framerate);
          m_stream_bin = new StreamBinHWEncoding(m_settings.src_framerate,
                                                 m_settings.stream_framerate,
                                                 m_settings.udp_destination);
        }
        else
        {
          m_source_bin = new SourceBinSWEnc();
          m_stream_bin = new StreamBinSWEncoding(m_settings.stream_framerate, m_settings.udp_destination);
        }

        if (m_settings.record_to_file)
        {
          if (m_settings.use_hw_encoding)
            m_record_bin = new RecordBinHWEncoding(createRecordingFilepath());
          else
            m_record_bin = new RecordBinSWEncoding(createRecordingFilepath());
        }
        else
        {
          m_record_bin = new RecordBinFakesink();
        }
      }

      bool
      createBins()
      {
        selectBins();

        if (!m_source_bin->create())
        {
          m_error = m_source_bin->error();
          return false;
        }

        if (!m_stream_bin->create())
        {
          m_error = m_stream_bin->error();
          return false;
        }

        if (!m_record_bin->create())
        {
          m_error = m_record_bin->error();
          return false;
        }

        return true;
      }

      bool
      addElementsToPipeline()
      {
        bool success {true};

        success &= gst_bin_add(GST_BIN(m_pipeline), m_appsrc->element());
        success &= gst_bin_add(GST_BIN(m_pipeline), m_source_bin->element());
        success &= gst_bin_add(GST_BIN(m_pipeline), m_tee->element());
        success &= gst_bin_add(GST_BIN(m_pipeline), m_stream_bin->element());
        success &= gst_bin_add(GST_BIN(m_pipeline), m_record_bin->element());

        if (!success)
          m_error = "failed to add all elements to the pipeline";

        return success;
      }

      bool
      linkPipeline()
      {
        if (!gst_element_link_filtered(m_appsrc->element(), m_source_bin->element(), m_appsrc->sourceCaps()))
        {
          m_error = "failed to link appsrc to bin '" + m_source_bin->name();
          return false;
        }

        if (!gst_element_link(m_source_bin->element(), m_tee->element()))
        {
          m_error = "failed to link bin '" + m_source_bin->name() + "' with tee";
          return false;
        }

        GstPad* tee_pad_1 = gst_element_get_request_pad(m_tee->element(), "src_%u");
        GstPad* tee_pad_2 = gst_element_get_request_pad(m_tee->element(), "src_%u");
        GstPad* record_pad = gst_element_get_static_pad(m_record_bin->element(), "sink");
        GstPad* stream_pad = gst_element_get_static_pad(m_stream_bin->element(), "sink");
        if (gst_pad_link(tee_pad_1, record_pad) != GST_PAD_LINK_OK ||
            gst_pad_link(tee_pad_2, stream_pad) != GST_PAD_LINK_OK)
        {
          m_error = "failed to link tee pads";
          return false;
        }

        return true;
      }

      bool
      checkReadyToStart()
      {
        bool ready = (m_pipeline != nullptr) &&
                     (m_bus != nullptr) &&
                     (m_appsrc != nullptr) &&
                     (m_source_bin != nullptr) &&
                     (m_record_bin != nullptr) &&
                     (m_stream_bin != nullptr) &&
                     (m_tee != nullptr);
        if (!ready)
          m_error = "pipeline not ready to start!";

        return ready;
      }

      bool
      setPipelinePlaying()
      {
        if (!gst_element_set_state(m_pipeline, GST_STATE_PLAYING))
        {
          m_error = "failed to start the pipeline";
          return false;
        }

        return true;
      }

      bool
      setPipelineStopping()
      {
        if (!gst_element_set_state(m_pipeline, GST_STATE_NULL))
        {
          m_error = "failed to stop the pipeline";
          return false;
        }

        return true;
      }

      std::string
      createRecordingFilepath()
      {
        std::string filename = "wic-recording-"
                               + Time::Format::getDateSafe()
                               + "-"
                               + Time::Format::getTimeSafe()
                               + ".avi";
        std::string filepath = (m_record_directory / filename).str();
        m_parent->inf("recording video to file: %s", filepath.c_str());
        return filepath;
      }

      void
      treatBusMsg()
      {
        GstMessage* msg = gst_bus_pop_filtered(
          m_bus,
          GstMessageType(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_EOS)
        );

        if (msg == nullptr)
          return;

        switch (GST_MESSAGE_TYPE(msg))
        {
          case GST_MESSAGE_ERROR:
          {
            GError* err;
            gchar* debug;
            gst_message_parse_error(msg, &err, &debug);
            m_parent->err("Gstreamer error: %s: %s", GST_OBJECT_NAME(msg->src), err->message);
            m_parent->debug("debug information: %s", debug);
            g_error_free(err);
            g_free(debug);
            break;
          }
          case GST_MESSAGE_WARNING:
          {
            GError* err;
            gchar* debug;
            gst_message_parse_warning(msg, &err, &debug);
            m_parent->war("Gstreamer warning: %s: %s", GST_OBJECT_NAME(msg->src), err->message);
            m_parent->debug("debug information: %s", debug);
            g_error_free(err);
            g_free(debug);
            break;
          }
          case GST_MESSAGE_STATE_CHANGED:
          {
            GstState old_state, new_state;
            gst_message_parse_state_changed (msg, &old_state, &new_state, nullptr);
            m_parent->debug("element state change: %-25s %s -> %s",
                             GST_OBJECT_NAME(msg->src),
                             gst_element_state_get_name(old_state),
                             gst_element_state_get_name(new_state));
            break;
          }
          case GST_MESSAGE_EOS:
          {
            m_parent->debug("received Gstreamer end-of-stream (EOS)");
            break;
          }
          default:
            break;
        }

        gst_message_unref(msg);
      }

      // Elements & Bins
      GstBus* m_bus;
      GstElement* m_pipeline;
      GstElementWrapper* m_tee;
      GstElementWrapper* m_appsrc;
      GstBinWrapper* m_source_bin;
      GstBinWrapper* m_record_bin;
      GstBinWrapper* m_stream_bin;

      //! Params & Properties
      PipelineSettings m_settings;
      FileSystem::Path m_record_directory;

      //! Other
      std::string m_error;
      Tasks::Task* m_parent;
    };
  }
}

#endif
