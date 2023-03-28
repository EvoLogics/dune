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

#ifndef SENSORS_WIC_BIN_HPP_INCLUDED_
#define SENSORS_WIC_BIN_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/DUNE.hpp>

// ISO C++ 11 headers.
#include <cstdint>
#include <string>

// Library headers.
#include <glib.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

// Local headers.
#include "Utils.hpp"

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    typedef std::vector<std::string> ErrorList;

    enum GhostPadType : uint8_t
    {
      SOURCE = 1U,
      SINK   = 2U,
      BOTH   = 3U,
    };

    struct Element
    {
      Element(GstElement* element, GstCaps* source_caps = nullptr):
        m_element {element},
        m_source_caps {source_caps}
      {
      }

      GstElement* m_element;
      GstCaps* m_source_caps;
    };

    class Bin
    {
    public:
      Bin(const std::string& name) :
        m_name {name}
      {
      }

      void
      setGhostPadType(const GhostPadType type)
      {
        m_ghost_pad_type = type;
      }

      GstElement*
      bin()
      {
        return m_bin;
      }

      std::string
      lastError()
      {
        return m_last_error;
      }

      std::string
      name()
      {
        return m_name;
      }

      bool
      create()
      {
        ErrorList errors;

        m_bin = gst_bin_new(m_name.c_str());

        if (!createElements(&errors))
        {
          m_last_error = String::str("bin '%s': failed to create following element(s): %s",
                                     m_name.c_str(), String::join(errors.begin(), errors.end(), ", ").c_str());
          return false;
        }

        if (!addElementsToBin(&errors))
        {
          m_last_error = String::str("bin '%s': failed to add following element(s): %s",
                                     m_name.c_str(), String::join(errors.begin(), errors.end(), ", ").c_str());
          return false;
        }

        if (!linkElements(&errors))
        {
          m_last_error = String::str("bin '%s': failed to create following links: %s",
                                     m_name.c_str(), String::join(errors.begin(), errors.end(), ", ").c_str());
          return false;
        }

        if (!createGhostPads())
        {
          m_last_error = String::str("failed to create ghost pads");
          return false;
        }

        return true;
      }

    protected:
      virtual
      bool
      createElements(ErrorList* errors) = 0;

      GstElement*
      createElement(const std::string& factory,
                    const std::string name,
                    ErrorList* errors)
      {
        GstElement* element = gst_element_factory_make(factory.c_str(), name.c_str());
        if (element == nullptr)
          errors->push_back(name);
        return element;
      }

      template <typename T>
      void
      setElementProperty(GstElement* element,
                         const std::string& name,
                         const T value)
      {
        if (element == nullptr)
          return;

        g_object_set(element, name.c_str(), value, nullptr);
      }

      bool
      addElementsToBin(ErrorList* errors)
      {
        for (Element element : m_elements)
        {
          GstElement* gst_element = element.m_element;
          if (!gst_bin_add(GST_BIN(m_bin), gst_element))
            errors->push_back(GST_ELEMENT_NAME(gst_element));
        }

        return errors->empty();
      }

      bool
      linkElements(ErrorList* errors)
      {
        if (m_elements.size() <= 1)
          return true;

        for (auto it = m_elements.begin(); it != std::prev(m_elements.end()) ; it++)
        {
          bool success = true;
          GstElement* current_element = (*it).m_element;
          GstCaps* current_source_caps = (*it).m_source_caps;
          GstElement* next_element = (*std::next(it)).m_element;

          if (current_source_caps != nullptr)
            success &= gst_element_link_filtered(current_element, next_element, current_source_caps);
          else
            success &= gst_element_link(current_element, next_element);

          if (!success)
            errors->push_back(String::str("%s->%s", GST_ELEMENT_NAME(current_element), GST_ELEMENT_NAME(next_element)));
        }

        return errors->empty();
      }

      bool
      createGhostPads()
      {
        bool success = true;

        if (m_ghost_pad_type & GhostPadType::SOURCE)
          success &= createGhostPad(true);

        if (m_ghost_pad_type & GhostPadType::SINK)
          success &= createGhostPad(false);

        return success;
      }

      bool
      createGhostPad(const bool ghost_src_pad)
      {
        const char* pad_name = ghost_src_pad ? "src" : "sink";
        GstElement* element = ghost_src_pad ? m_elements.back().m_element : m_elements.front().m_element;

        GstPad* pad = gst_element_get_static_pad(element, pad_name);
        GstPad* ghost_pad = gst_ghost_pad_new(pad_name, pad);
        gst_pad_set_active(ghost_pad, TRUE);
        const bool success = gst_element_add_pad(m_bin, ghost_pad);
        gst_object_unref(pad);

        return success;
      }

      std::vector<Element> m_elements;
      GstElement* m_bin;
      GhostPadType m_ghost_pad_type;
      std::string m_name;
      std::string m_last_error;
    };


    class SourceBinSWEnc : public Bin
    {
    public:
      SourceBinSWEnc():
        Bin("source_bin_sw_encoding")
      {
        setGhostPadType(GhostPadType::BOTH);
      }

    private:
      bool
      createElements(ErrorList* errors)
      {
        GstElement* source_converter = createElement("videoconvert", "source_converter", errors);
        if (!errors->empty())
          return false;

        m_elements.emplace_back(source_converter, nullptr);
        return true;
      }
    };


    class SourceBinHWEnc : public Bin
    {
    public:
      SourceBinHWEnc(const int frame_width, const int frame_height, const int framerate):
        Bin("source_bin_hw_encoding"),
        m_frame_width {frame_width},
        m_frame_height {frame_height},
        m_framerate {framerate}
      {
        setGhostPadType(GhostPadType::BOTH);
      }

    private:
      bool
      createElements(ErrorList* errors)
      {
        GstElement* source_converter = createElement("videoconvert", "source_converter", errors);
        GstElement* source_nv_converter = createElement("nvvidconv", "source_nv_converter", errors);
        if (!errors->empty())
          return false;

        GstCaps* converter_caps = gst_caps_new_simple("video/x-raw",
                                                      "format", G_TYPE_STRING, "I420",
                                                      "width", G_TYPE_INT, m_frame_width,
                                                      "height", G_TYPE_INT, m_frame_height,
                                                      "framerate", GST_TYPE_FRACTION, m_framerate, 1,
                                                      nullptr);
        GstCaps *nv_converter_caps =
          gst_caps_from_string(String::str("video/x-raw(memory:NVMM), format=NV12, width=%d, height=%d, framerate=%d/1",
                                           m_frame_width,
                                           m_frame_height,
                                           m_framerate).c_str());
        m_elements.emplace_back(source_converter, converter_caps);
        m_elements.emplace_back(source_nv_converter, nv_converter_caps);
        return true;
      }

      int m_frame_width;
      int m_frame_height;
      int m_framerate;
    };


    class RecordBinFakesink : public Bin
    {
    public:
      RecordBinFakesink():
        Bin("record_bin_fakesink")
      {
        setGhostPadType(GhostPadType::SINK);
      }

    private:
      bool
      createElements(ErrorList* errors)
      {
        GstElement* record_queue = createElement("queue", "record_queue", errors);
        GstElement* record_fakesink = createElement("fakesink", "record_fakesink", errors);
        if (!errors->empty())
          return false;

        m_elements.emplace_back(record_queue, nullptr);
        m_elements.emplace_back(record_fakesink, nullptr);
        return true;
      }
    };


    class RecordBinSWEncoding : public Bin
    {
    public:
      RecordBinSWEncoding(const Path& save_location):
        Bin("record_bin_sw_encoding"),
        m_save_location {save_location}
      {
        setGhostPadType(GhostPadType::SINK);
      }

    private:
      bool
      createElements(ErrorList* errors)
      {
        GstElement* record_queue = createElement("queue", "record_queue", errors);
        GstElement* record_encoder = createElement("x264enc", "record_encoder", errors);
        GstElement* record_parser = createElement("h264parse", "record_parser", errors);
        GstElement* record_mux = createElement("avimux", "record_mux", errors);
        GstElement* record_sink = createElement("filesink", "record_sink", errors);
        setElementProperty(record_encoder, "tune", c_pipeline_x264enc_tune);
        setElementProperty(record_sink, "location", m_save_location.str().c_str());
        if (!errors->empty())
          return false;

        m_elements.emplace_back(record_queue, nullptr);
        m_elements.emplace_back(record_encoder, nullptr);
        m_elements.emplace_back(record_parser, nullptr);
        m_elements.emplace_back(record_mux, nullptr);
        m_elements.emplace_back(record_sink, nullptr);
        return true;
      }

      Path m_save_location;
    };


    class StreamBinSWEncoding : public Bin
    {
    public:
      StreamBinSWEncoding(const std::string& udp_destination):
        Bin("stream_bin_sw_encoding"),
        m_udp_destination {udp_destination}
      {
        setGhostPadType(GhostPadType::SINK);
      }

    private:
      bool
      createElements(ErrorList* errors)
      {
        GstElement* stream_queue = createElement("queue", "stream_queue", errors);
        GstElement* stream_videorate = createElement("videorate", "stream_videorate", errors);
        GstElement* stream_encoder = createElement("x264enc", "stream_encoder", errors);
        GstElement* stream_parser = createElement("h264parse", "stream_parser", errors);
        GstElement* stream_payloader = createElement("rtph264pay", "stream_payloader", errors);
        GstElement* stream_sink = createElement("udpsink", "stream_sink", errors);
        setElementProperty(stream_videorate, "max-rate", c_pipeline_videorate_max_rate);
        setElementProperty(stream_encoder, "tune", c_pipeline_x264enc_tune);
        setElementProperty(stream_encoder, "speed-preset", c_pipeline_x264enc_speed_preset);
        setElementProperty(stream_encoder, "qp-min", c_pipeline_x264enc_qp_min);
        setElementProperty(stream_encoder, "key-int-max", c_pipeline_x264enc_key_int_max);
        setElementProperty(stream_sink, "host", m_udp_destination.c_str());
        if (!errors->empty())
          return false;

        m_elements.emplace_back(stream_queue, nullptr);
        m_elements.emplace_back(stream_videorate, nullptr);
        m_elements.emplace_back(stream_encoder, nullptr);
        m_elements.emplace_back(stream_parser, nullptr);
        m_elements.emplace_back(stream_payloader, nullptr);
        m_elements.emplace_back(stream_sink, nullptr);
        return true;
      }

      std::string m_udp_destination;
    };


    class RecordBinHWEncoding : public Bin
    {
    public:
      RecordBinHWEncoding(const int framerate, const Path& save_location):
        Bin("record_bin_hw_encoding"),
        m_framerate {framerate},
        m_save_location {save_location}
      {
        setGhostPadType(GhostPadType::SINK);
      }

    private:
      bool
      createElements(ErrorList* errors)
      {
        GstElement* record_queue = createElement("queue", "record_queue", errors);
        GstElement* record_encoder = createElement("nvv4l2h264enc", "record_encoder", errors);
        GstElement* record_parser = createElement("h264parse", "record_parser", errors);
        GstElement* record_mux = createElement("avimux", "record_mux", errors);
        GstElement* record_sink = createElement("filesink", "record_sink", errors);
        setElementProperty(record_encoder, "bitrate", c_pipeline_nvv4l2h264enc_bitrate);
        setElementProperty(record_encoder, "insert-sps-pps", c_pipeline_nvv4l2h264enc_insert_sps_pps);
        setElementProperty(record_encoder, "iframeinterval", m_framerate);
        setElementProperty(record_encoder, "maxperf-enable", c_pipeline_nvv4l2h264enc_maxperf_enable);
        setElementProperty(record_sink, "location", m_save_location.str().c_str());
        if (!errors->empty())
          return false;

        m_elements.emplace_back(record_queue, nullptr);
        m_elements.emplace_back(record_encoder, nullptr);
        m_elements.emplace_back(record_parser, nullptr);
        m_elements.emplace_back(record_mux, nullptr);
        m_elements.emplace_back(record_sink, nullptr);
        return true;
      }

      int m_framerate;
      Path m_save_location;
    };


    class StreamBinHWEncoding : public Bin
    {
    public:
      StreamBinHWEncoding(const int framerate, const std::string& udp_destination):
        Bin("stream_bin_sw_encoding"),
        m_framerate {framerate},
        m_udp_destination {udp_destination}
      {
        setGhostPadType(GhostPadType::SINK);
      }

    private:
      bool
      createElements(ErrorList* errors)
      {
        GstElement* stream_queue = createElement("queue", "stream_queue", errors);
        GstElement* stream_videorate = createElement("videorate", "stream_videorate", errors);
        GstElement* stream_encoder = createElement("nvv4l2h264enc", "stream_encoder", errors);
        GstElement* stream_parser = createElement("h264parse", "stream_parser", errors);
        GstElement* stream_payloader = createElement("rtph264pay", "stream_payloader", errors);
        GstElement* stream_sink = createElement("udpsink", "stream_sink", errors);
        setElementProperty(stream_videorate, "max-rate", c_pipeline_videorate_max_rate);
        setElementProperty(stream_encoder, "bitrate", c_pipeline_nvv4l2h264enc_bitrate);
        setElementProperty(stream_encoder, "insert-sps-pps", c_pipeline_nvv4l2h264enc_insert_sps_pps);
        setElementProperty(stream_encoder, "iframeinterval", m_framerate);
        setElementProperty(stream_encoder, "maxperf-enable", c_pipeline_nvv4l2h264enc_maxperf_enable);
        setElementProperty(stream_sink, "host", m_udp_destination.c_str());
        if (!errors->empty())
          return false;

        m_elements.emplace_back(stream_queue, nullptr);
        m_elements.emplace_back(stream_videorate, nullptr);
        m_elements.emplace_back(stream_encoder, nullptr);
        m_elements.emplace_back(stream_parser, nullptr);
        m_elements.emplace_back(stream_payloader, nullptr);
        m_elements.emplace_back(stream_sink, nullptr);
        return true;
      }

      int m_framerate;
      std::string m_udp_destination;
    };
  }
}

#endif

