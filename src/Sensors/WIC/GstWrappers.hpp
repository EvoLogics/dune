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

#ifndef SENSORS_WIC_GST_WRAPPERS_HPP_INCLUDED_
#define SENSORS_WIC_GST_WRAPPERS_HPP_INCLUDED_

// C++ STL headers.
#include <string>
#include <vector>

// Library headers.
#include <glib.h>
#include <gst/gst.h>

// Local headers.
#include "Constants.hpp"

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    //! Convenience typedef.
    typedef std::vector<std::string> ErrorList;

    //! Enum indicating which type of ghostpad a GST bin uses.
    //! (see also: https://gstreamer.freedesktop.org/documentation/gstreamer/gstghostpad.html?gi-language=c).
    enum GstGhostPadType : unsigned
    {
      SOURCE = 1U,
      SINK   = 2U,
      BOTH   = 3U,
    };

    class GstElementWrapper
    {
    public:
      GstElementWrapper(const std::string& name, const std::string& factory_name):
        element_name {name},
        source_caps {nullptr}
      {
        gst_element = gst_element_factory_make(factory_name.c_str(), name.c_str());
        is_valid = (gst_element != nullptr);
      }

      std::string
      name() const
      {
        return element_name;
      }

      bool
      valid() const
      {
        return is_valid;
      }

      GstElement*
      element() const
      {
        return gst_element;
      }

      GstCaps*
      sourceCaps() const
      {
        return source_caps;
      }

      void
      setSourceCaps(const std::string& caps_string)
      {
        source_caps = gst_caps_from_string(caps_string.c_str());
        is_valid &= (source_caps != nullptr);
      }

      template <typename T>
      void
      setProperty(const std::string& property_name, const T value)
      {
        if (gst_element != nullptr)
          g_object_set(gst_element, property_name.c_str(), value, nullptr);
      }

    private:
      std::string element_name;
      GstElement* gst_element;
      GstCaps* source_caps;
      bool is_valid;
    };

    class GstBinWrapper
    {
    public:
      GstBinWrapper(const std::string& name, const GstGhostPadType type) :
        m_name {name},
        m_ghost_pad_type {type}
      {
        m_bin = gst_bin_new(m_name.c_str());
      }

      GstElement*
      element() const
      {
        return m_bin;
      }

      std::string
      name() const
      {
        return m_name;
      }

      std::string
      error() const
      {
        return m_error;
      }

      bool
      create()
      {
        ErrorList errors;

        createElements();

        if (!validateElements(errors))
        {
          m_error = String::str("bin '%s': failed to create following element(s): %s",
                                m_name.c_str(), String::join(errors.begin(), errors.end(), ", ").c_str());
          return false;
        }

        if (!addElementsToBin(errors))
        {
          m_error = String::str("bin '%s': failed to add following element(s): %s",
                                m_name.c_str(), String::join(errors.begin(), errors.end(), ", ").c_str());
          return false;
        }

        if (!linkElements(errors))
        {
          m_error = String::str("bin '%s': failed to create following links: %s",
                                m_name.c_str(), String::join(errors.begin(), errors.end(), ", ").c_str());
          return false;
        }

        if (!createGhostPads())
        {
          m_error = String::str("bin '%s': failed to create ghost pads", m_name.c_str());
          return false;
        }

        return true;
      }

    protected:
      virtual void
      createElements() = 0;

      void
      addElement(const GstElementWrapper element)
      {
        m_elements.push_back(element);
      }

    private:
      bool
      validateElements(ErrorList& errors)
      {
        for (GstElementWrapper element : m_elements)
          if (!element.valid())
            errors.push_back(element.name());
        return errors.empty();
      }

      bool
      addElementsToBin(ErrorList& errors)
      {
        for (GstElementWrapper element : m_elements)
          if (!gst_bin_add(GST_BIN(m_bin), element.element()))
            errors.push_back(element.name());
        return errors.empty();
      }

      bool
      linkElements(ErrorList& errors)
      {
        if (m_elements.size() <= 1U)
          return true;

        for (auto it = m_elements.begin(); it != std::prev(m_elements.end()) ; it++)
        {
          GstElement* element = it->element();
          GstElement* next_element = std::next(it)->element();
          GstCaps* source_caps = it->sourceCaps();

          bool success = false;
          if (source_caps != nullptr)
            success = gst_element_link_filtered(element, next_element, source_caps);
          else
            success = gst_element_link(element, next_element);

          if (!success)
            errors.push_back(String::str("%s->%s", it->name().c_str(), std::next(it)->name().c_str()));
        }

        return errors.empty();
      }

      bool
      createGhostPads()
      {
        bool success = true;

        if (m_ghost_pad_type & GstGhostPadType::SOURCE)
          success &= createGhostPad(true);

        if (m_ghost_pad_type & GstGhostPadType::SINK)
          success &= createGhostPad(false);

        return success;
      }

      bool
      createGhostPad(const bool ghost_src_pad)
      {
        const char* pad_name = ghost_src_pad ? "src" : "sink";
        GstElement* element = ghost_src_pad ? m_elements.back().element() : m_elements.front().element();

        GstPad* pad = gst_element_get_static_pad(element, pad_name);
        GstPad* ghost_pad = gst_ghost_pad_new(pad_name, pad);
        gst_pad_set_active(ghost_pad, true);
        const bool success = gst_element_add_pad(m_bin, ghost_pad);

        gst_object_unref(pad);

        return success;
      }

      std::vector<GstElementWrapper> m_elements;
      std::string m_name;
      std::string m_error;
      GstElement* m_bin;
      GstGhostPadType m_ghost_pad_type;
    };

    class SourceBinSWEnc : public GstBinWrapper
    {
    public:
      SourceBinSWEnc():
        GstBinWrapper("source_bin_sw_encoding", GstGhostPadType::BOTH)
      {
      }

    private:
      void
      createElements()
      {
        GstElementWrapper source_converter {"source_converter", "videoconvert"};
        addElement(source_converter);
      }
    };

    class SourceBinHWEnc : public GstBinWrapper
    {
    public:
      SourceBinHWEnc(const int frame_width, const int frame_height, const int framerate):
        GstBinWrapper("source_bin_hw_encoding", GstGhostPadType::BOTH),
        m_frame_width {frame_width},
        m_frame_height {frame_height},
        m_framerate {framerate}
      {
      }

    private:
      void
      createElements()
      {
        GstElementWrapper converter {"source_converter", "videoconvert"};
        GstElementWrapper nv_converter {"source_nv_converter", "nvvidconv"};

        converter.setSourceCaps(String::str("video/x-raw, format=I420, width=%d, height=%d, framerate=%d/1",
                                            m_frame_width, m_frame_height, m_framerate).c_str());
        nv_converter.setSourceCaps(String::str("video/x-raw(memory:NVMM), format=NV12, width=%d, height=%d, framerate=%d/1",
                                               m_frame_width, m_frame_height, m_framerate).c_str());

        addElement(converter);
        addElement(nv_converter);
      }

      int m_frame_width;
      int m_frame_height;
      int m_framerate;
    };

    class RecordBinFakesink : public GstBinWrapper
    {
    public:
      RecordBinFakesink():
        GstBinWrapper("record_bin_fakesink", GstGhostPadType::SINK)
      {
      }

    private:
      void
      createElements()
      {
        GstElementWrapper record_queue {"record_queue", "queue"};
        GstElementWrapper record_fakesink {"record_fakesink", "fakesink"};

        addElement(record_queue);
        addElement(record_fakesink);
      }
    };

    class RecordBinSWEncoding : public GstBinWrapper
    {
    public:
      RecordBinSWEncoding(const FileSystem::Path& save_location):
        GstBinWrapper("record_bin_sw_encoding", GstGhostPadType::SINK),
        m_save_location {save_location}
      {
      }

    private:
      void
      createElements()
      {
        GstElementWrapper record_queue {"record_queue", "queue"};
        GstElementWrapper record_encoder {"record_encoder", "x264enc"};
        GstElementWrapper record_parser {"record_parser", "h264parse"};
        GstElementWrapper record_mux {"record_mux", "avimux"};
        GstElementWrapper record_sink {"record_sink", "filesink"};

        record_queue.setProperty("max-size-buffers", c_pipeline_queue_record_max_size_buffers);
        record_encoder.setProperty("tune", c_pipeline_x264enc_tune);
        record_encoder.setProperty("speed-preset", c_pipeline_x264enc_speed_preset);
        record_encoder.setProperty("key-int-max", c_pipeline_x264enc_record_key_int_max);
        record_encoder.setProperty("bitrate", c_pipeline_x264enc_bitrate);
        record_sink.setProperty("location", m_save_location.str().c_str());

        addElement(record_queue);
        addElement(record_encoder);
        addElement(record_parser);
        addElement(record_mux);
        addElement(record_sink);
      }

      FileSystem::Path m_save_location;
    };

    class StreamBinSWEncoding : public GstBinWrapper
    {
    public:
      StreamBinSWEncoding(const int stream_framerate, const std::string& udp_destination):
        GstBinWrapper("stream_bin_sw_encoding", GstGhostPadType::SINK),
        m_udp_destination {udp_destination},
        m_stream_framerate {stream_framerate}
      {
      }

    private:
      void
      createElements()
      {
        GstElementWrapper stream_queue {"stream_queue", "queue"};
        GstElementWrapper stream_videorate {"stream_videorate", "videorate"};
        GstElementWrapper stream_encoder {"stream_encoder", "x264enc"};
        GstElementWrapper stream_parser {"stream_parser", "h264parse"};
        GstElementWrapper stream_payloader {"stream_payloader", "rtph264pay"};
        GstElementWrapper stream_sink {"stream_sink", "udpsink"};

        stream_queue.setProperty("max-size-buffers", c_pipeline_queue_stream_max_size_buffers);
        stream_videorate.setProperty("max-rate", m_stream_framerate);
        stream_encoder.setProperty("tune", c_pipeline_x264enc_tune);
        stream_encoder.setProperty("speed-preset", c_pipeline_x264enc_speed_preset);
        stream_encoder.setProperty("bitrate", c_pipeline_x264enc_bitrate);
        stream_encoder.setProperty("key-int-max", c_pipeline_x264enc_stream_key_int_max);
        stream_payloader.setProperty("config-interval", c_pipeline_stream_payloader_config_interval);
        stream_sink.setProperty("host", m_udp_destination.c_str());

        addElement(stream_queue);
        addElement(stream_videorate);
        addElement(stream_encoder);
        addElement(stream_parser);
        addElement(stream_payloader);
        addElement(stream_sink);
      }

      std::string m_udp_destination;
      int m_stream_framerate;
    };

    class RecordBinHWEncoding : public GstBinWrapper
    {
    public:
      RecordBinHWEncoding(const Path& save_location):
        GstBinWrapper("record_bin_hw_encoding", GstGhostPadType::SINK),
        m_save_location {save_location}
      {
      }

    private:
      void
      createElements()
      {
        GstElementWrapper record_queue {"record_queue", "queue"};
        GstElementWrapper record_encoder {"record_encoder", "nvv4l2h264enc"};
        GstElementWrapper record_parser {"record_parser", "h264parse"};
        GstElementWrapper record_mux {"record_mux", "avimux"};
        GstElementWrapper record_sink {"record_sink", "filesink"};

        record_queue.setProperty("max-size-buffers", c_pipeline_queue_record_max_size_buffers);
        record_encoder.setProperty("bitrate", c_pipeline_nvv4l2h264enc_bitrate);
        record_encoder.setProperty("insert-sps-pps", c_pipeline_nvv4l2h264enc_insert_sps_pps);
        record_encoder.setProperty("iframeinterval", 1);
        record_encoder.setProperty("maxperf-enable", c_pipeline_nvv4l2h264enc_maxperf_enable);
        record_sink.setProperty("location", m_save_location.str().c_str());

        addElement(record_queue);
        addElement(record_encoder);
        addElement(record_parser);
        addElement(record_mux);
        addElement(record_sink);
      }

      Path m_save_location;
    };

    class StreamBinHWEncoding : public GstBinWrapper
    {
    public:
      StreamBinHWEncoding(const int framerate, const int stream_framerate, const std::string& udp_destination):
        GstBinWrapper("stream_bin_hw_encoding", GstGhostPadType::SINK),
        m_framerate {framerate},
        m_stream_framerate {stream_framerate},
        m_udp_destination {udp_destination}
      {
      }

    private:
      void
      createElements()
      {
        GstElementWrapper stream_queue {"stream_queue", "queue"};
        GstElementWrapper stream_videorate {"stream_videorate", "videorate"};
        GstElementWrapper stream_encoder {"stream_encoder", "nvv4l2h264enc"};
        GstElementWrapper stream_parser {"stream_parser", "h264parse"};
        GstElementWrapper stream_payloader {"stream_payloader", "rtph264pay"};
        GstElementWrapper stream_sink {"stream_sink", "udpsink"};

        stream_queue.setProperty("max-size-buffers", c_pipeline_queue_stream_max_size_buffers);
        stream_videorate.setProperty("max-rate", m_stream_framerate);
        stream_encoder.setProperty("bitrate", c_pipeline_nvv4l2h264enc_bitrate);
        stream_encoder.setProperty("insert-sps-pps", c_pipeline_nvv4l2h264enc_insert_sps_pps);
        stream_encoder.setProperty("iframeinterval", m_framerate);
        stream_encoder.setProperty("maxperf-enable", c_pipeline_nvv4l2h264enc_maxperf_enable);
        stream_payloader.setProperty("config-interval", c_pipeline_stream_payloader_config_interval);
        stream_sink.setProperty("host", m_udp_destination.c_str());

        addElement(stream_queue);
        addElement(stream_videorate);
        addElement(stream_encoder);
        addElement(stream_parser);
        addElement(stream_payloader);
        addElement(stream_sink);
      }

      int m_framerate;
      int m_stream_framerate;
      std::string m_udp_destination;
    };
  }
}

#endif
