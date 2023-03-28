/* //*************************************************************************** */
/* // Copyright 2007-2023 EvoLogics GmbH                                       * */
/* //*************************************************************************** */
/* // This file is part of DUNE: Unified Navigation Environment.               * */
/* //                                                                          * */
/* // Commercial Licence Usage                                                 * */
/* // Licencees holding valid commercial DUNE licences may use this file in    * */
/* // accordance with the commercial licence agreement provided with the       * */
/* // Software or, alternatively, in accordance with the terms contained in a  * */
/* // written agreement between you and Universidade do Porto. For licensing   * */
/* // terms, conditions, and further information contact lsts@fe.up.pt.        * */
/* //                                                                          * */
/* // European Union Public Licence - EUPL v.1.1 Usage                         * */
/* // Alternatively, this file may be used under the terms of the EUPL,        * */
/* // Version 1.1 only (the "Licence"), appearing in the file LICENCE.md       * */
/* // included in the packaging of this file. You may not use this work        * */
/* // except in compliance with the Licence. Unless required by applicable     * */
/* // law or agreed to in writing, software distributed under the Licence is   * */
/* // distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     * */
/* // ANY KIND, either express or implied. See the Licence for the specific    * */
/* // language governing permissions and limitations at                        * */
/* // http://ec.europa.eu/idabc/eupl.html.                                     * */
/* //*************************************************************************** */
/* // Author: Michael Purser                                                   * */
/* //*************************************************************************** */

/* // DUNE headers. */
/* #include <DUNE/DUNE.hpp> */

/* // ISO C++ 11 headers. */
/* #include <string> */

/* // Library headers. */
/* #include <glib.h> */
/* #include <gst/gst.h> */
/* #include <gst/app/gstappsrc.h> */

/* // Local headers. */
/* #include "Types.hpp" */
/* #include "Utils.hpp" */

/* namespace Sensors */
/* { */
/*   namespace WIC */
/*   { */
/*     using DUNE_NAMESPACES; */

/*     struct StreamerSettings */
/*     { */
/*       bool use_hw_encoding; */
/*       bool record; */
/*       LogDirectory save_location {"", false}; */
/*       size_t src_width; */
/*       size_t src_height; */
/*       size_t src_framerate; */
/*     }; */

/*     class Streamer */
/*     { */
/*     public: */
/*       Streamer(Task* parent): */
/*         m_parent {parent}, */
/*         m_settings {false, false, {}, 0, 0, 0}, */
/*         m_pipeline {nullptr}, */
/*         m_restart_needed {false}, */
/*         m_running {false}, */
/*         m_take_picture {false} */
/*       { */
/*       } */

/*       bool */
/*       running() */
/*       { */
/*         // TODO */
/*         return m_running; */
/*       } */

/*       std::string */
/*       lastError() */
/*       { */
/*         return m_last_error; */
/*       } */

/*       Path */
/*       getSaveLocation() */
/*       { */
/*         return m_settings.save_location.location; */
/*       } */

/*       void */
/*       updateUseHardwareEncoding(const bool use_hw_encoding) */
/*       { */
/*         spew(use_hw_encoding ? "using hardware encoding" : "using software encoding"); */
/*         m_settings.use_hw_encoding = use_hw_encoding; */
/*       } */

/*       void */
/*       updateRecord(const bool record) */
/*       { */
/*         spew(record ? "recording video stream" : "not recording video stream"); */
/*         m_settings.record = record; */
/*       } */

/*       void */
/*       updateSaveLocation(const Path& path) */
/*       { */
/*         spew("save location set to " + path.str()); */
/*         m_settings.save_location.update(path); */
/*       } */

/*       void */
/*       initialize(const bool use_hardware_encoding, */
/*                  const bool record, */
/*                  const size_t width, */
/*                  const size_t height, */
/*                  const size_t framerate) */
/*       { */
/*         spew("initializing"); */

/*         gst_init(nullptr, nullptr); */

/*         m_settings.use_hw_encoding = use_hardware_encoding; */
/*         m_settings.record = record; */
/*         m_settings.src_width = width; */
/*         m_settings.src_height = height; */
/*         m_settings.src_framerate = framerate; */
/*       } */

/*       void */
/*       requestRestart() */
/*       { */
/*         spew("requesting restart"); */
/*         m_restart_needed = true; */
/*       } */

/*       bool */
/*       restartIfRequested() */
/*       { */
/*         if (!m_restart_needed) */
/*           return true; */
/*         m_restart_needed = false; */

/*         if (!m_settings.save_location.set_first_time) */
/*         { */
/*           m_last_error = "no directory for saving recordings/pictures was defined"; */
/*           return false; */
/*         } */

/*         // Always allow continuing after stop to not block in some weird state. */
/*         if (!stop()) */
/*           m_parent->war("%s", m_last_error.c_str()); */

/*         cleanup(); */

/*         if (!create()) */
/*           return false; */

/*         if (!start()) */
/*           return false; */

/*         return true; */
/*       } */

/*       /1* bool *1/ */
/*       /1* create() *1/ */
/*       /1* { *1/ */
/*       /1*   spew("creating the pipeline"); *1/ */

/*       /1*   m_pipeline = gst_pipeline_new("pipeline"); *1/ */
/*       /1*   m_bus = gst_pipeline_get_bus(GST_PIPELINE(m_pipeline)); *1/ */

/*       /1*   PipelineBin* source_bin = getSourceBin(); *1/ */
/*       /1*   PipelineBin* record_bin = getRecordBin(); *1/ */
/*       /1*   PipelineBin* stream_bin = getStreamBin(); *1/ */
/*       /1*   GstElement* tee; *1/ */

/*       /1*   if (!createBinAndAddToPipeline(source_bin) || *1/ */
/*       /1*       !createBinAndAddToPipeline(record_bin) || *1/ */
/*       /1*       !createBinAndAddToPipeline(stream_bin)) *1/ */
/*       /1*     return false; *1/ */

/*       /1*   if (!createTeeElementAndAddToBin(&tee)) *1/ */
/*       /1*     return false; *1/ */

/*       /1*   if (!linkPipeline(source_bin, record_bin, stream_bin, &tee)) *1/ */
/*       /1*     return false; *1/ */

/*       /1*   return true; *1/ */
/*       /1* } *1/ */

/*       /1* PipelineBin* *1/ */
/*       /1* getSourceBin() *1/ */
/*       /1* { *1/ */
/*       /1*   SourceBinSWEnc* source = new SourceBinSWEnc(m_parent); *1/ */
/*       /1*   return source; *1/ */
/*       /1* } *1/ */

/*       /1* PipelineBin* *1/ */
/*       /1* getRecordBin() *1/ */
/*       /1* { *1/ */
/*       /1*   SinkBinRecordFakesink* record = new SinkBinRecordFakesink(m_parent); *1/ */
/*       /1*   return record; *1/ */
/*       /1* } *1/ */

/*       /1* PipelineBin* *1/ */
/*       /1* getStreamBin() *1/ */
/*       /1* { *1/ */
/*       /1*   SinkBinStreamFakesink* stream = new SinkBinStreamFakesink(m_parent); *1/ */
/*       /1*   return stream; *1/ */
/*       /1* } *1/ */

/*       /1* bool *1/ */
/*       /1* createTeeElementAndAddToBin(GstElement** tee) *1/ */
/*       /1* { *1/ */
/*       /1*   *tee = gst_element_factory_make("tee", "tee"); *1/ */
/*       /1*   if (tee == nullptr) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_last_error = "failed to create tee element"; *1/ */
/*       /1*     return false; *1/ */
/*       /1*   } *1/ */

/*       /1*   if (!gst_bin_add(GST_BIN(m_pipeline), *tee)) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_last_error = "failed to add tee to pipeline"; *1/ */
/*       /1*     return false; *1/ */
/*       /1*   } *1/ */

/*       /1*   return true; *1/ */
/*       /1* } *1/ */

/*       /1* bool *1/ */
/*       /1* createBinAndAddToPipeline(PipelineBin* bin) *1/ */
/*       /1* { *1/ */
/*       /1*   if (!bin->create()) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_last_error = bin->m_last_error; *1/ */
/*       /1*     return false; *1/ */
/*       /1*   } *1/ */

/*       /1*   if (!gst_bin_add(GST_BIN(m_pipeline), bin->m_bin)) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_last_error = String::str("failed to add bin %s to pipeline", GST_ELEMENT_NAME(bin->m_bin)); *1/ */
/*       /1*     return false; *1/ */
/*       /1*   } *1/ */

/*       /1*   return true; *1/ */
/*       /1* } *1/ */

/*       /1* bool *1/ */
/*       /1* linkPipeline(PipelineBin* source_bin, PipelineBin* record_bin, PipelineBin* stream_bin, GstElement** tee) *1/ */
/*       /1* { *1/ */
/*       /1*   if (!gst_element_link(source_bin->m_bin, *tee)) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_last_error = "failed to link tee with source bin"; *1/ */
/*       /1*     return false; *1/ */
/*       /1*   } *1/ */

/*       /1*   GstPad* tee_pad_1 = gst_element_get_request_pad(*tee, "src_%u"); *1/ */
/*       /1*   GstPad* tee_pad_2 = gst_element_get_request_pad(*tee, "src_%u"); *1/ */
/*       /1*   GstPad* record_pad = gst_element_get_static_pad(record_bin->m_bin, "sink"); *1/ */
/*       /1*   GstPad* stream_pad = gst_element_get_static_pad(stream_bin->m_bin, "sink"); *1/ */
/*       /1*   if (gst_pad_link(tee_pad_1, record_pad) != GST_PAD_LINK_OK || *1/ */
/*       /1*       gst_pad_link(tee_pad_2, stream_pad) != GST_PAD_LINK_OK) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_last_error = "failed to link tee pads"; *1/ */
/*       /1*     return false; *1/ */
/*       /1*   } *1/ */

/*       /1*   return true; *1/ */
/*       /1* } *1/ */

/*       /1* bool *1/ */
/*       /1* start() *1/ */
/*       /1* { *1/ */
/*       /1*   spew("starting the pipeline"); *1/ */
/*       /1*   if (!gst_element_set_state(m_pipeline, GST_STATE_PLAYING)) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_last_error = "failed to start the pipeline"; *1/ */
/*       /1*     return false; *1/ */
/*       /1*   } *1/ */
/*       /1*   return true; *1/ */
/*       /1* } *1/ */

/*       /1* bool *1/ */
/*       /1* stop() *1/ */
/*       /1* { *1/ */
/*       /1*   if (m_pipeline == nullptr) *1/ */
/*       /1*     return true; *1/ */

/*       /1*   spew("stopping the pipeline"); *1/ */
/*       /1*   if (!gst_element_set_state(m_pipeline, GST_STATE_NULL)) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_last_error = "failed to stop the pipeline"; *1/ */
/*       /1*     return false; *1/ */
/*       /1*   } *1/ */
/*       /1*   return true; *1/ */
/*       /1* } *1/ */

/*       /1* void *1/ */
/*       /1* cleanup() *1/ */
/*       /1* { *1/ */
/*       /1*   spew("cleaning up"); *1/ */

/*       /1*   unreferencePipeline(); *1/ */
/*       /1*   unreferenceBus(); *1/ */
/*       /1* } *1/ */

/*       /1* void *1/ */
/*       /1* unreferencePipeline() *1/ */
/*       /1* { *1/ */
/*       /1*   if (m_pipeline != nullptr) *1/ */
/*       /1*   { *1/ */
/*       /1*     spew("unreferencing pipeline"); *1/ */
/*       /1*     gst_object_unref(m_pipeline); *1/ */
/*       /1*     m_pipeline = nullptr; *1/ */
/*       /1*   } *1/ */
/*       /1* } *1/ */

/*       /1* void *1/ */
/*       /1* unreferenceBus() *1/ */
/*       /1* { *1/ */
/*       /1*   if (m_bus != nullptr) *1/ */
/*       /1*   { *1/ */
/*       /1*     spew("unreferencing bus"); *1/ */
/*       /1*     gst_object_unref(m_bus); *1/ */
/*       /1*     m_bus = nullptr; *1/ */
/*       /1*   } *1/ */
/*       /1* } *1/ */

/*       void */
/*       requestTakePicture() */
/*       { */
/*         spew("requesting take picture"); */
/*         m_take_picture = true; */
/*       } */

/*       bool */
/*       takePictureIfRequested(/*uint8_t* data*/) */
/*       { */
/*         if (!m_take_picture) */
/*           return true; */

/*         m_take_picture = false; */

/*         spew(String::str("taking picture")); */
/*         // TODO */
/*         // Use member vars for width/height */
/*         return true; */
/*       } */

/*       void */
/*       pushCameraFrame(uint8_t* data) */
/*       { */
/*         // TODO */
/*         // Use member variables width and height and constant 2 bytes per pixel for size determination. */
/*       } */

/*       /1* void *1/ */
/*       /1* getGstBusMessages() *1/ */
/*       /1* { *1/ */
/*       /1*   GstMessage* msg = gst_bus_pop_filtered(m_bus, *1/ */
/*       /1*       GstMessageType(GST_MESSAGE_ERROR | GST_MESSAGE_WARNING | GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_EOS)); *1/ */

/*       /1*   if (msg == nullptr) *1/ */
/*       /1*     return; *1/ */

/*       /1*   switch (GST_MESSAGE_TYPE(msg)) *1/ */
/*       /1*   { *1/ */
/*       /1*     case GST_MESSAGE_ERROR: *1/ */
/*       /1*     { *1/ */
/*       /1*       GError* err; *1/ */
/*       /1*       gchar* debug; *1/ */
/*       /1*       gst_message_parse_error(msg, &err, &debug); *1/ */
/*       /1*       m_parent->err("Gstreamer error: %-15s: %s", GST_OBJECT_NAME(msg->src), err->message); *1/ */
/*       /1*       spew(String::str("debug information: %s", debug)); *1/ */
/*       /1*       g_error_free(err); *1/ */
/*       /1*       g_free(debug); *1/ */
/*       /1*       break; *1/ */
/*       /1*     } *1/ */
/*       /1*     case GST_MESSAGE_WARNING: *1/ */
/*       /1*     { *1/ */
/*       /1*       GError* err; *1/ */
/*       /1*       gchar* debug; *1/ */
/*       /1*       gst_message_parse_warning(msg, &err, &debug); *1/ */
/*       /1*       m_parent->war("Gstreamer warning: %-15s: %s", GST_OBJECT_NAME(msg->src), err->message); *1/ */
/*       /1*       spew(String::str("debug information: %s", debug)); *1/ */
/*       /1*       g_error_free(err); *1/ */
/*       /1*       g_free(debug); *1/ */
/*       /1*       break; *1/ */
/*       /1*     } *1/ */
/*       /1*     case GST_MESSAGE_STATE_CHANGED: *1/ */
/*       /1*     { *1/ */
/*       /1*       GstState old_state, new_state; *1/ */
/*       /1*       gst_message_parse_state_changed (msg, &old_state, &new_state, nullptr); *1/ */
/*       /1*       spew(String::str("element state change: %-25s %s -> %s", *1/ */
/*       /1*                        GST_OBJECT_NAME(msg->src), *1/ */
/*       /1*                        gst_element_state_get_name(old_state), *1/ */
/*       /1*                        gst_element_state_get_name(new_state))); *1/ */
/*       /1*       break; *1/ */
/*       /1*     } *1/ */
/*       /1*     case GST_MESSAGE_EOS: *1/ */
/*       /1*       spew("Gstreamer end-of-stream received"); *1/ */
/*       /1*       break; *1/ */
/*       /1*     default: *1/ */
/*       /1*       break; *1/ */
/*       /1*   } *1/ */

/*       /1*   gst_message_unref(msg); *1/ */
/*       /1* } *1/ */

/*       /1* void *1/ */
/*       /1* constructPipeline() *1/ */
/*       /1* { *1/ */
/*       /1*   spew("Constructing pipeline"); *1/ */

/*       /1*   m_pipeline = gst_pipeline_new("pipeline"); *1/ */

/*       /1*   m_bus = gst_pipeline_get_bus(GST_PIPELINE(m_pipeline)); *1/ */

/*       /1*   PipelineElements elements; *1/ */
/*       /1*   /2* createPipelineElement(&elements.source, "v4l2src", "source"); *2/ *1/ */
/*       /1*   createPipelineElement(&m_appsrc, "appsrc", "appsrc"); *1/ */
/*       /1*   /2* createPipelineElement(&elements.source_capsfilter, "capsfilter", "source_capsfilter"); *2/ *1/ */
/*       /1*   createPipelineElement(&elements.tee, "tee", "tee"); *1/ */
/*       /1*   createPipelineElement(&elements.queue_1, "queue", "queue_1"); *1/ */
/*       /1*   createPipelineElement(&elements.queue_2, "queue", "queue_2"); *1/ */
/*       /1*   createPipelineElement(&elements.video_converter, "videoconvert", "video_converter"); *1/ */
/*       /1*   createPipelineElement(&elements.nv_converter, "nvvidconv", "nv_converter"); *1/ */
/*       /1*   createPipelineElement(&elements.nv_encoder, "nvv4l2h264enc", "nv_encoder"); *1/ */
/*       /1*   createPipelineElement(&elements.h264_parser, "h264parse", "h264_parser"); *1/ */
/*       /1*   createPipelineElement(&elements.nv_encoder_2, "nvv4l2h264enc", "nv_encoder_2"); *1/ */
/*       /1*   createPipelineElement(&elements.h264_parser_2, "h264parse", "h264_parser_2"); *1/ */
/*       /1*   createPipelineElement(&elements.videorate_filter, "videorate", "videorate_filter"); *1/ */
/*       /1*   /2* createPipelineElement(&elements.h264_encoder_1, "nvv4l2h264enc", "h264_encoder_1"); *2/ *1/ */
/*       /1*   createPipelineElement(&elements.h264_payloader, "rtph264pay", "h264_payloader"); *1/ */
/*       /1*   createPipelineElement(&elements.udp_sink, "udpsink", "udp_sink"); *1/ */
/*       /1*   /2* createPipelineElement(&elements.videoconvert_filter_2, "videoconvert", "videoconvert_filter_2"); *2/ *1/ */
/*       /1*   /2* createPipelineElement(&elements.h264_encoder_2, "nvv4l2h264enc", "h264_encoder_2"); *2/ *1/ */
/*       /1*   createPipelineElement(&elements.avimux, "avimux", "avimux"); *1/ */
/*       /1*   createPipelineElement(&elements.filesink, "filesink", "filesink"); *1/ */
/*       /1*   /2* createPipelineElement(&elements.autovideosink, "autovideosink", "autovideosink"); *2/ *1/ */
/*       /1*   /2* createPipelineElement(&elements.fakesink, "fakesink", "fakesink"); *2/ *1/ */

/*       /1*   // Configure elements *1/ */
/*       /1*   // TODO: move some of these to config *1/ */
/*       /1*   /2* GstCaps* source_caps = gst_caps_new_full(gst_structure_new("video/x-raw", *2/ *1/ */
/*       /1*   /2*                                                            "format", G_TYPE_STRING, "YUY2", *2/ *1/ */
/*       /1*   /2*                                                            "width", G_TYPE_INT, 640, *2/ *1/ */
/*       /1*   /2*                                                            "height", G_TYPE_INT, 360, *2/ *1/ */
/*       /1*   /2*                                                            "framerate", GST_TYPE_FRACTION, 30, 1, *2/ *1/ */
/*       /1*   /2*                                                            nullptr), *2/ *1/ */
/*       /1*   /2*                                          nullptr); *2/ *1/ */
/*       /1*   /2* g_object_set(elements.source_capsfilter, "caps", source_caps, nullptr); *2/ *1/ */

/*       /1*   if (m_appsrc == nullptr) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_parent->err("Failed to create appsrc element"); *1/ */
/*       /1*     return; *1/ */
/*       /1*   } *1/ */

/*       /1*   // Configure the appsrc element *1/ */
/*       /1*   g_object_set(m_appsrc, *1/ */
/*       /1*     "stream-type", 0, *1/ */
/*       /1*     "format", GST_FORMAT_TIME, *1/ */
/*       /1*     "is-live", true, *1/ */
/*       /1*     "do-timestamp", true, *1/ */
/*       /1*     "max-buffers", 1, *1/ */
/*       /1*     nullptr); *1/ */

/*       /1*   // Configure the nv encoder *1/ */
/*       /1*   g_object_set(elements.nv_encoder, *1/ */
/*       /1*     "bitrate", 157286400, *1/ */
/*       /1*     "insert-sps-pps", 1, *1/ */
/*       /1*     "iframeinterval", 30, *1/ */
/*       /1*     "maxperf-enable", true, *1/ */
/*       /1*     nullptr); *1/ */

/*       /1*   // Configure the nv encoder *1/ */
/*       /1*   g_object_set(elements.nv_encoder_2, *1/ */
/*       /1*     "bitrate", 157286400, *1/ */
/*       /1*     "insert-sps-pps", 1, *1/ */
/*       /1*     "iframeinterval", 30, *1/ */
/*       /1*     "maxperf-enable", true, *1/ */
/*       /1*     nullptr); *1/ */

/*       /1*   GstCaps *cap_appsrc_to_video_converter = *1/ */
/*       /1*     gst_caps_new_simple("video/x-raw", *1/ */
/*       /1*                         "format", G_TYPE_STRING, "GRAY16_BE", *1/ */
/*       /1*                         "width", G_TYPE_INT, 640, *1/ */
/*       /1*                         "height", G_TYPE_INT, 512, *1/ */
/*       /1*                         "framerate", GST_TYPE_FRACTION, 30, 1, *1/ */
/*       /1*                         nullptr); *1/ */

/*       /1*   GstCaps *cap_video_converter_to_nv_converter = *1/ */
/*       /1*     gst_caps_new_simple("video/x-raw", *1/ */
/*       /1*                         "format", G_TYPE_STRING, "I420", *1/ */
/*       /1*                         "width", G_TYPE_INT, 640, *1/ */
/*       /1*                         "height", G_TYPE_INT, 512, *1/ */
/*       /1*                         "framerate", GST_TYPE_FRACTION, 30, 1, *1/ */
/*       /1*                         nullptr); *1/ */

/*       /1*   GstCaps *cap_nv_converter_to_tee = *1/ */
/*       /1*     gst_caps_from_string("video/x-raw(memory:NVMM), format=NV12, width=640, height=512, framerate=30/1"); *1/ */
/*       /1*                         /2* "width", G_TYPE_INT, 640, *2/ *1/ */
/*       /1*                         /2* "height", G_TYPE_INT, 512, *2/ *1/ */
/*       /1*                         /2* "framerate", GST_TYPE_FRACTION, 30, 1, *2/ *1/ */
/*       /1*                         /2* nullptr); *2/ *1/ */

/*       /1*   g_object_set(elements.videorate_filter, "max-rate", 10, nullptr); *1/ */
/*       /1*   /2* g_object_set(elements.h264_encoder_1, "tune", 0x00000004, nullptr); *2/ *1/ */
/*       /1*   /2* g_object_set(elements.h264_encoder_1, "speed-preset", 0x00000001, nullptr); *2/ *1/ */
/*       /1*   /2* g_object_set(elements.h264_encoder_1, "qp-min", 20, nullptr); *2/ *1/ */
/*       /1*   /2* g_object_set(elements.h264_encoder_1, "key-int-max", 1, nullptr); *2/ *1/ */
/*       /1*   g_object_set(elements.udp_sink, "host", "192.168.3.153", nullptr); *1/ */
/*       /1*   /2* g_object_set(elements.h264_encoder_2, "tune", 0x00000004, nullptr); *2/ *1/ */
/*       /1*   g_object_set(elements.filesink, "location", "test.avi", nullptr); *1/ */
/*       /1*   /2* g_object_set(elements.queue_1, "max-size-buffers", 1, nullptr); *2/ *1/ */
/*       /1*   /2* g_object_set(elements.queue_2, "max-size-buffers", 1, nullptr); *2/ *1/ */
/*       /1*   /2* g_object_set(elements.filesink, "sync", "false", nullptr); *2/ *1/ */
/*       /1*   /2* g_object_set(elements.autovideosink, "sync", "false", nullptr); *2/ *1/ */

/*       /1*   // Add the elements to the pipeline *1/ */
/*       /1*   /2* addElementToPipeline(elements.source); *2/ *1/ */
/*       /1*   addElementToPipeline(m_appsrc); *1/ */
/*       /1*   /2* addElementToPipeline(elements.source_capsfilter); *2/ *1/ */
/*       /1*   addElementToPipeline(elements.tee); *1/ */
/*       /1*   addElementToPipeline(elements.queue_1); *1/ */
/*       /1*   addElementToPipeline(elements.queue_2); *1/ */
/*       /1*   addElementToPipeline(elements.video_converter); *1/ */
/*       /1*   addElementToPipeline(elements.nv_converter); *1/ */
/*       /1*   addElementToPipeline(elements.nv_encoder); *1/ */
/*       /1*   addElementToPipeline(elements.h264_parser); *1/ */
/*       /1*   addElementToPipeline(elements.nv_encoder_2); *1/ */
/*       /1*   addElementToPipeline(elements.h264_parser_2); *1/ */
/*       /1*   /2* addElementToPipeline(elements.videorate_filter); *2/ *1/ */
/*       /1*   /2* addElementToPipeline(elements.h264_encoder_1); *2/ *1/ */
/*       /1*   addElementToPipeline(elements.h264_payloader); *1/ */
/*       /1*   addElementToPipeline(elements.udp_sink); *1/ */
/*       /1*   /2* addElementToPipeline(elements.videoconvert_filter_2); *2/ *1/ */
/*       /1*   /2* addElementToPipeline(elements.h264_encoder_2); *2/ *1/ */
/*       /1*   addElementToPipeline(elements.avimux); *1/ */
/*       /1*   addElementToPipeline(elements.filesink); *1/ */
/*       /1*   /2* addElementToPipeline(elements.autovideosink); *2/ *1/ */
/*       /1*   /2* addElementToPipeline(elements.fakesink); *2/ *1/ */

/*       /1*   // Link the elements *1/ */
/*       /1*   spew("Linking pipeline elements"); *1/ */

/*       /1*   spew("Linking appsrc to video converter"); *1/ */
/*       /1*   bool caps_link_success_1 = gst_element_link_filtered( *1/ */
/*       /1*                                                        m_appsrc, *1/ */
/*       /1*                                                        elements.video_converter, *1/ */
/*       /1*                                                        cap_appsrc_to_video_converter *1/ */
/*       /1*                                                       ); *1/ */
/*       /1*   if (!caps_link_success_1) *1/ */
/*       /1*     m_parent->err("Failed to link appsrc to video converter"); *1/ */

/*       /1*   spew("Linking video converter to nv converter"); *1/ */
/*       /1*   bool caps_link_success_2 = gst_element_link_filtered( *1/ */
/*       /1*                                                        elements.video_converter, *1/ */
/*       /1*                                                        elements.nv_converter, *1/ */
/*       /1*                                                        cap_video_converter_to_nv_converter *1/ */
/*       /1*                                                       ); *1/ */
/*       /1*   if (!caps_link_success_2) *1/ */
/*       /1*     m_parent->err("Failed to link video converter to nv converter"); *1/ */

/*       /1*   spew("Linking nv converter to tee"); *1/ */
/*       /1*   bool tee_link_success = gst_element_link_filtered( *1/ */
/*       /1*                                                     elements.nv_converter, *1/ */
/*       /1*                                                     elements.tee, *1/ */
/*       /1*                                                     cap_nv_converter_to_tee *1/ */
/*       /1*                                                    ); *1/ */
/*       /1*   if (!tee_link_success) *1/ */
/*       /1*     m_parent->err("Failed to link videoconvert_filter_1 to tee"); *1/ */

/*       /1*   spew("Linking queue_1 branch"); *1/ */
/*       /1*   bool stream_branch_link_success = gst_element_link_many( *1/ */
/*       /1*                                                           elements.queue_1, *1/ */
/*       /1*                                                           /2* elements.videorate_filter, *2/ *1/ */
/*       /1*                                                           /2* elements.h264_encoder_1, *2/ *1/ */
/*       /1*                                                           elements.nv_encoder, *1/ */
/*       /1*                                                           elements.h264_parser, *1/ */
/*       /1*                                                           elements.h264_payloader, *1/ */
/*       /1*                                                           elements.udp_sink, *1/ */
/*       /1*                                                           nullptr *1/ */
/*       /1*                                                           ); *1/ */
/*       /1*   if (!stream_branch_link_success) *1/ */
/*       /1*     m_parent->err("Failed to link queue_1 branch"); *1/ */

/*       /1*   spew("Linking queue_2 branch"); *1/ */
/*       /1*   bool display_branch_link_success = gst_element_link_many( *1/ */
/*       /1*                                                            elements.queue_2, *1/ */
/*       /1*                                                            /2* elements.h264_encoder_2, *2/ *1/ */
/*       /1*                                                            elements.nv_encoder_2, *1/ */
/*       /1*                                                            elements.h264_parser_2, *1/ */
/*       /1*                                                            elements.avimux, *1/ */
/*       /1*                                                            elements.filesink, *1/ */
/*       /1*                                                            /2* elements.fakesink, *2/ *1/ */
/*       /1*                                                            nullptr *1/ */
/*       /1*                                                            ); *1/ */
/*       /1*   if (!display_branch_link_success) *1/ */
/*       /1*     m_parent->err("Failed to link queue_2 branch"); *1/ */

/*       /1*   /2* if (!caps_link_success_1 || !tee_link_success || !stream_branch_link_success || !display_branch_link_success) *2/ *1/ */
/*       /1*   /2*   m_parent->err("Failed to link pipeline elements"); *2/ *1/ */

/*       /1*   // Manual linking of tee pads *1/ */
/*       /1*   GstPad* tee_pad_1 = gst_element_get_request_pad(elements.tee, "src_%u"); *1/ */
/*       /1*   GstPad* tee_pad_2 = gst_element_get_request_pad(elements.tee, "src_%u"); *1/ */
/*       /1*   GstPad* queue_1_pad = gst_element_get_static_pad(elements.queue_1, "sink"); *1/ */
/*       /1*   GstPad* queue_2_pad = gst_element_get_static_pad(elements.queue_2, "sink"); *1/ */
/*       /1*   if ( *1/ */
/*       /1*       gst_pad_link (tee_pad_1, queue_1_pad) != GST_PAD_LINK_OK || *1/ */
/*       /1*       gst_pad_link (tee_pad_2, queue_2_pad) != GST_PAD_LINK_OK *1/ */
/*       /1*      ) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_parent->err("Failed to link tee pads to queues"); *1/ */
/*       /1*   } *1/ */
/*       /1* } *1/ */

/*       void */
/*       spin() */
/*       { */
/*         while (gst_bus_peek(m_bus) != nullptr) */
/*           getGstBusMessages(); */
/*       } */

/*       /1* void *1/ */
/*       /1* pushSrcData(uint8_t* data, unsigned size) *1/ */
/*       /1* { *1/ */
/*       /1*   // TODO: check inputs *1/ */

/*       /1*   GstCaps* caps; *1/ */
/*       /1*   GstSample* sample; *1/ */
/*       /1*   GstBuffer* buffer; *1/ */
/*       /1*   /2* unsigned size; *2/ *1/ */
/*       /1*   GstFlowReturn ret; *1/ */

/*       /1*   constexpr unsigned height = 512; *1/ */
/*       /1*   constexpr unsigned width = 640; *1/ */
/*       /1*   /2* constexpr unsigned bytes_per_pixel = 1; *2/ *1/ */

/*       /1*   /2* size = height * width * bytes_per_pixel; *2/ *1/ */

/*       /1*   /2* spew("Creating caps"); *2/ *1/ */
/*       /1*   caps = gst_caps_new_simple("video/x-raw", *1/ */
/*       /1*                              "format", G_TYPE_STRING, "GRAY16_BE", *1/ */
/*       /1*                              "width", G_TYPE_INT, width, *1/ */
/*       /1*                              "height", G_TYPE_INT, height, *1/ */
/*       /1*                              "framerate", GST_TYPE_FRACTION, 30, 1, *1/ */
/*       /1*                              nullptr); *1/ */
/*       /1*   if (caps == nullptr) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_parent->err("Failed to create caps"); *1/ */
/*       /1*     return; *1/ */
/*       /1*   } *1/ */

/*       /1*   /2* spew(String::str("Allocating buffer of size %d", size)); *2/ *1/ */
/*       /1*   buffer = gst_buffer_new_allocate(nullptr, size, nullptr); *1/ */
/*       /1*   if (buffer == nullptr) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_parent->err("Failed to allocate buffer"); *1/ */
/*       /1*     return; *1/ */
/*       /1*   } *1/ */

/*       /1*   /2* size_t num_filled = gst_buffer_memset(buffer, 0, m_white_image ? 150U : 100U, size); *2/ *1/ */
/*       /1*   size_t num_filled = gst_buffer_fill(buffer, 0, data, size); *1/ */
/*       /1*   /2* GST_BUFFER_PTS(buffer) = m_timestamp; *2/ *1/ */
/*       /1*   /2* GstClock* clock = gst_system_clock_obtain(); *2/ *1/ */
/*       /1*   /2* GST_BUFFER_PTS(buffer) = gst_clock_get_time(clock); *2/ *1/ */
/*       /1*   /2* GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 30); *2/ *1/ */
/*       /1*   /2* spew(String::str("Buffer timestamp: %d", GST_BUFFER_PTS(buffer))); *2/ *1/ */
/*       /1*   /2* spew(String::str("Buffer duration: %d", GST_BUFFER_DURATION(buffer))); *2/ *1/ */
/*       /1*   /2* m_timestamp += GST_BUFFER_DURATION(buffer); *2/ *1/ */
/*       /1*   /2* spew(String::str("Filled %d bytes of buffer", num_filled)); *2/ *1/ */

/*       /1*   /2* m_white_image = !m_white_image; *2/ *1/ */

/*       /1*   /2* spew("Creating sample"); *2/ *1/ */
/*       /1*   sample = gst_sample_new(buffer, caps, nullptr, nullptr); *1/ */
/*       /1*   if (sample == nullptr) *1/ */
/*       /1*   { *1/ */
/*       /1*     m_parent->err("Failed to create sample"); *1/ */
/*       /1*     return; *1/ */
/*       /1*   } *1/ */

/*       /1*   m_white_image = !m_white_image; *1/ */


/*       /1*   /2* ret = gst_app_src_push_buffer(GST_APP_SRC_CAST(m_appsrc), buffer); *2/ *1/ */

/*       /1*   /2* spew("Pushing sample to appsrc"); *2/ *1/ */
/*       /1*   ret = gst_app_src_push_sample(GST_APP_SRC_CAST(m_appsrc), sample); *1/ */
/*       /1*   if (ret != GST_FLOW_OK) *1/ */
/*       /1*     m_parent->err("Failed to push buffer to appsrc: %d", ret); *1/ */

/*       /1*   gst_sample_unref(sample); *1/ */
/*       /1*   gst_buffer_unref(buffer); *1/ */
/*       /1* } *1/ */

/*     private: */
/*       void */
/*       spew(const std::string& msg) */
/*       { */
/*         /1* m_parent->spew("%s [ms]: [%-8s] %s", String::str(Time::Clock::getMsec()).c_str(), "Streamer", msg.c_str()); *1/ */
/*         m_parent->spew("[%-8s] %s", "Streamer", msg.c_str()); */
/*       } */

/*       Task* m_parent; */
/*       StreamerSettings m_settings; */
/*       GstBus* m_bus; */
/*       GstElement* m_pipeline; */
/*       GstElement* m_appsrc; */
/*       bool m_restart_needed; */
/*       bool m_running; */
/*       bool m_take_picture; */
/*       std::string m_last_error; */
/*     }; */
/*   } */
/* } */
