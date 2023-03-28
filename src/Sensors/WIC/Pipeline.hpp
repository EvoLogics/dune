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

// DUNE headers.
#include <DUNE/DUNE.hpp>

// ISO C++ 11 headers.
#include <string>

// Library headers.
#include <glib.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// Local headers.
#include "Bin.hpp"
#include "Utils.hpp"

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    enum PipelineState : uint8_t
    {
      NOT_PLAYING,
      PLAYING,
      ERROR,
    };

    struct PipelineSettings
    {
      bool use_hw_encoding;
      bool record_to_file;
      int source_frame_width;
      int source_frame_height;
      int source_framerate;
      std::string udp_destination;
    };

    class Pipeline
    {
    public:
      Pipeline(Tasks::Task* parent):
        m_record_file_counter {0U},
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
          m_parent->inf("set hw encoding to '%s'", settings.use_hw_encoding ? "true" : "false"); 
        if (settings.record_to_file != m_settings.record_to_file)
          m_parent->inf("set record to file to '%s'", settings.record_to_file ? "true" : "false");
        if (settings.source_frame_width != m_settings.source_frame_width)
          m_parent->inf("set source frame width to %d", settings.source_frame_width);
        if (settings.source_frame_height != m_settings.source_frame_height)
          m_parent->inf("set source frame height to %d", settings.source_frame_height);
        if (settings.source_framerate != m_settings.source_framerate)
          m_parent->inf("set source framerate to %d", settings.source_framerate);
        if (settings.udp_destination != m_settings.udp_destination)
          m_parent->inf("set UDP destination to %s", settings.udp_destination.c_str());

        m_settings = settings;
      }

      void
      setSaveLocation(const Path& save_location)
      {
        if (save_location != m_save_location)
        {
          m_parent->inf("set save location to %s", save_location.str().c_str());
          m_save_location = save_location;
          m_record_file_counter = 0U;
        }
      }

      std::string
      error()
      {
        return m_error;
      }

      PipelineState
      state()
      {
        if (!m_error.empty())
          return PipelineState::ERROR;
        if (m_pipeline == nullptr)
          return PipelineState::NOT_PLAYING;
        return (GST_STATE(m_pipeline) == GST_STATE_PLAYING) ? PipelineState::PLAYING : PipelineState::NOT_PLAYING;
      }

      Path
      saveLocation()
      {
        return m_save_location;
      }

      void
      cleanup()
      {
        if (m_pipeline != nullptr)
        {
          spew("unreferencing pipeline");
          gst_object_unref(m_pipeline);
          m_pipeline = nullptr;
        }
        if (m_bus != nullptr)
        {
          spew("unreferencing bus");
          gst_object_unref(m_bus);
          m_bus = nullptr;
        }
      }

      bool
      createAndStart()
      {
        if (!create())
          return false;
        if (!start())
          return false;
        return true;
      }

      bool
      create()
      {
        spew("creating pipeline");

        if (m_settings.record_to_file && m_save_location.type() == Path::Type::PT_INVALID)
        {
          m_error = "failed to create pipeline: invalid save location: '" + m_save_location.str() + "'";
          return false;
        }

        spew("initializing pipeline and bus objects");
        m_pipeline = gst_pipeline_new("pipeline");
        m_bus = gst_pipeline_get_bus(GST_PIPELINE(m_pipeline));

        spew("selecting bins");
        selectBinsBasedOnConfig();

        if (m_source_bin == nullptr || m_record_bin == nullptr || m_stream_bin == nullptr)
        {
          m_error = "not all bins are defined - something went wrong during bin selection";
          return false;
        }

        spew("creating appsrc");
        if (!createAppsrcAndAddToPipeline())
          return false;

        spew("creating bins");
        if (!createBinAndAddToPipeline(m_source_bin) ||
            !createBinAndAddToPipeline(m_record_bin) ||
            !createBinAndAddToPipeline(m_stream_bin))
          return false;

        spew("creating tee");
        if (!createTeeAndAddToPipeline())
          return false;

        spew("linking pipeline");
        if (!linkPipeline())
          return false;

        resetError();
        return true;
      }

      bool
      start()
      {
        spew("starting pipeline");
        if (!readyToStart())
        {
          m_error = "not all elements were created!";
          return false;
        }
        if (!gst_element_set_state(m_pipeline, GST_STATE_PLAYING))
        {
          m_error = "failed to start the pipeline";
          return false;
        }
        resetError();
        return true;
      }

      bool
      stop()
      {
        if (m_pipeline == nullptr)
          return true;

        spew("stopping the pipeline");
        if (!gst_element_set_state(m_pipeline, GST_STATE_NULL))
        {
          m_error = "failed to stop the pipeline";
          return false;
        }
        resetError();
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

        const size_t data_size = m_settings.source_frame_width * m_settings.source_frame_height * 2U; 

        buffer = gst_buffer_new_allocate(nullptr, data_size, nullptr);
        if (buffer == nullptr)
          return;

        size_t num_filled = gst_buffer_fill(buffer, 0, data, data_size);
        if (num_filled != data_size)
          return;

        sample = gst_sample_new(buffer, m_appsrc->m_source_caps, nullptr, nullptr);
        if (sample == nullptr)
          return;

        ret = gst_app_src_push_sample(GST_APP_SRC_CAST(m_appsrc->m_element), sample);
        if (ret != GST_FLOW_OK)
          return;

        gst_sample_unref(sample);
        gst_buffer_unref(buffer);
      }

    private:
      bool
      readyToStart()
      {
        return (m_pipeline != nullptr) &&
               (m_bus != nullptr) &&
               (m_source_bin != nullptr) &&
               (m_record_bin != nullptr) &&
               (m_stream_bin != nullptr);
      }

      void
      selectBinsBasedOnConfig()
      {
        if (m_settings.use_hw_encoding)
        {
          m_source_bin = new SourceBinHWEnc(m_settings.source_frame_width,
                                            m_settings.source_frame_height,
                                            m_settings.source_framerate);
          if (m_settings.record_to_file)
            m_record_bin = new RecordBinHWEncoding(m_settings.source_framerate, currentRecordingFilename());
          else
            m_record_bin = new RecordBinFakesink();
          m_stream_bin = new StreamBinHWEncoding(m_settings.source_framerate, m_settings.udp_destination);
        }
        else
        {
          m_source_bin = new SourceBinSWEnc();
          if (m_settings.record_to_file)
            m_record_bin = new RecordBinSWEncoding(currentRecordingFilename());
          else
            m_record_bin = new RecordBinFakesink();
          m_stream_bin = new StreamBinSWEncoding(m_settings.udp_destination);
        }
      }

      std::string
      currentRecordingFilename()
      {
        m_record_file_counter += 1U;
        return (m_save_location/String::str("wic-recording-%lu.avi", m_record_file_counter)).str();
      }

      bool
      createAppsrcAndAddToPipeline()
      {
        spew("creating appsrc element & caps");
        GstElement* appsrc_element = gst_element_factory_make("appsrc", "source");
        GstCaps* appsrc_caps = gst_caps_new_simple("video/x-raw",
                                                   "format", G_TYPE_STRING, "GRAY16_BE",
                                                   "width", G_TYPE_INT, m_settings.source_frame_width,
                                                   "height", G_TYPE_INT, m_settings.source_frame_height,
                                                   "framerate", GST_TYPE_FRACTION, m_settings.source_framerate, 1,
                                                   nullptr);

        if (appsrc_element == nullptr)
        {
          m_error = "failed to create appsrc element";
          return false;
        }

        spew("assigning appsrc element & caps");
        m_appsrc = new Element(appsrc_element, appsrc_caps);

        spew("setting appsrc properties");
        g_object_set(m_appsrc->m_element,
                     "stream-type", 0,
                     "format", GST_FORMAT_TIME,
                     "is-live", true,
                     "do-timestamp", true,
                     nullptr);

        spew("adding appsrc to pipeline");
        if (!gst_bin_add(GST_BIN(m_pipeline), m_appsrc->m_element))
        {
          m_error = "failed to add appsrc to pipeline";
          return false;
        }

        resetError();
        return true;
      }

      bool
      createTeeAndAddToPipeline()
      {
        m_tee = gst_element_factory_make("tee", "tee");
        if (m_tee == nullptr)
        {
          m_error = "failed to create tee element";
          return false;
        }

        if (!gst_bin_add(GST_BIN(m_pipeline), m_tee))
        {
          m_error = "failed to add tee to pipeline";
          return false;
        }

        resetError();
        return true;
      }

      bool
      createBinAndAddToPipeline(Bin* bin)
      {
        spew(String::str("creating bin '%s'", bin->name().c_str()));
        if (!bin->create())
        {
          m_error = bin->lastError();
          return false;
        }
        spew(String::str("created bin '%s'", bin->name().c_str()));

        spew(String::str("adding bin '%s' to pipeline", bin->name().c_str()));
        if (!gst_bin_add(GST_BIN(m_pipeline), bin->bin()))
        {
          m_error = String::str("failed to add bin %s to pipeline", GST_ELEMENT_NAME(bin->bin()));
          return false;
        }
        spew(String::str("added bin '%s' to pipeline", bin->name().c_str()));

        resetError();
        return true;
      }

      bool
      linkPipeline()
      {
        spew(String::str("linking appsrc with bin '%s'", m_source_bin->name().c_str()));
        if (!gst_element_link_filtered(m_appsrc->m_element, m_source_bin->bin(), m_appsrc->m_source_caps))
        {
          m_error = "failed to link appsrc to bin '" + m_source_bin->name();
          return false;
        }

        spew(String::str("linking bin '%s' with tee", m_source_bin->name().c_str()));
        if (!gst_element_link(m_source_bin->bin(), m_tee))
        {
          m_error = "failed to link bin '" + m_source_bin->name() + "' with tee";
          return false;
        }

        spew("linking tee pads");
        GstPad* tee_pad_1 = gst_element_get_request_pad(m_tee, "src_%u");
        GstPad* tee_pad_2 = gst_element_get_request_pad(m_tee, "src_%u");
        GstPad* record_pad = gst_element_get_static_pad(m_record_bin->bin(), "sink");
        GstPad* stream_pad = gst_element_get_static_pad(m_stream_bin->bin(), "sink");
        if (gst_pad_link(tee_pad_1, record_pad) != GST_PAD_LINK_OK ||
            gst_pad_link(tee_pad_2, stream_pad) != GST_PAD_LINK_OK)
        {
          m_error = "failed to link tee pads";
          return false;
        }

        resetError();
        return true;
      }

      void
      treatBusMsg()
      {
        GstMessage* msg = gst_bus_pop_filtered(m_bus,
          GstMessageType(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_EOS));

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
            spew(String::str("debug information: %s", debug));
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
            spew(String::str("debug information: %s", debug));
            g_error_free(err);
            g_free(debug);
            break;
          }
          case GST_MESSAGE_STATE_CHANGED:
          {
            GstState old_state, new_state;
            gst_message_parse_state_changed (msg, &old_state, &new_state, nullptr);
            spew(String::str("element state change: %-25s %s -> %s",
                             GST_OBJECT_NAME(msg->src),
                             gst_element_state_get_name(old_state),
                             gst_element_state_get_name(new_state)));
            break;
          }
          case GST_MESSAGE_EOS:
            spew("Gstreamer end-of-stream received");
            break;
          default:
            break;
        }

        gst_message_unref(msg);
      }

      void
      resetError()
      {
        m_error.clear();
      }

      void
      spew(const std::string& msg)
      {
        m_parent->spew("[%-8s] %s", "Pipeline", msg.c_str());
      }

      // Elements & Bins
      GstBus* m_bus;
      GstElement* m_pipeline;
      GstElement* m_tee;
      Element* m_appsrc;
      Bin* m_source_bin;
      Bin* m_record_bin;
      Bin* m_stream_bin;

      //! Params & Properties
      PipelineSettings m_settings;
      Path m_save_location;
      size_t m_record_file_counter;

      //! Other
      std::string m_error;
      Tasks::Task* m_parent;
    };
  }
}

#endif
