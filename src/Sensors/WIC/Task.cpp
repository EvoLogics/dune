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

// DUNE headers.
#include <DUNE/DUNE.hpp>

// ISO C++ 11 headers.
#include <string>

// Local headers.
#include "CameraInterface.hpp"
#include "Constants.hpp"
#include "DUNE/Status/Codes.hpp"
#include "Pipeline.hpp"

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    enum Error : unsigned
    {
      NO_CAMERA_DATA,
      PIPELINE_ERROR,
      PIPELINE_TIMEOUT,
    };

    struct Arguments
    {
      std::string camera_license_file_location;
      std::string camera_range_mode;
      std::string camera_ffc_mode;
      std::string camera_digital_output_depth;
      std::string camera_palette;
      uint16_t camera_brightness;
      std::string camera_agc_type;
      std::string camera_video_color_mode;
      bool camera_invert_image_horizontally;
      bool camera_invert_image_vertically;
      bool use_hw_encoding;
      bool record_to_file;
      std::string udp_destination;
    };

    struct Task: public DUNE::Tasks::Task
    {
      Arguments m_args;
      CameraInterface m_camera_if;
      LogCollector m_errors;
      Pipeline m_pipeline;
      bool m_restart_pipeline;
      bool m_take_picture;
      Counter<double> m_pipeline_restart_timer;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_camera_if {this},
        m_pipeline {this},
        m_restart_pipeline {false},
        m_take_picture {false},
        m_pipeline_restart_timer {c_pipeline_start_timeout}
      {
        setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_INIT);

        setStackSize(c_thread_stack_size);
        inf("increased task stack size to: %lu", getStackSize());

        param("Camera License File Location", m_args.camera_license_file_location)
        .defaultValue("/")
        .description("Location of the camera license file")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param("Camera Range Mode", m_args.camera_range_mode)
        .values(String::str("%s,%s,%s",
                            WIC_RANGE_LOW,
                            WIC_RANGE_MIDDLE,
                            WIC_RANGE_HIGH))
        .defaultValue(WIC_RANGE_LOW)
        .description("Camera range mode")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param("Camera FFC Mode", m_args.camera_ffc_mode)
        .values(String::str("%s,%s,%s",
                            WIC_FFC_MANUAL,
                            WIC_FFC_AUTO,
                            WIC_FFC_EXTERNAL))
        .defaultValue(WIC_FFC_MANUAL)
        .description("Camera FFC mode")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param("Camera Digital Output Depth", m_args.camera_digital_output_depth)
        .values(String::str("%s,%s,%s,%s",
                            WIC_DOD_8_BIT_GRAYSCALE,
                            WIC_DOD_8_BIT_BAYER,
                            WIC_DOD_14_BIT_RAW,
                            WIC_DOD_16_BIT_YCBCR))
        .defaultValue(WIC_DOD_16_BIT_YCBCR)
        .description("Camera digital output depth")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param("Camera Palette", m_args.camera_palette)
        .values(String::str("%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
                            WIC_PALETTE_WHITE_HOT,
                            WIC_PALETTE_BLACK_HOT,
                            WIC_PALETTE_FUSION,
                            WIC_PALETTE_RAINBOW,
                            WIC_PALETTE_GLOBOW,
                            WIC_PALETTE_IRONBOW1,
                            WIC_PALETTE_IRONBOW2,
                            WIC_PALETTE_SEPIA,
                            WIC_PALETTE_COLOR1,
                            WIC_PALETTE_COLOR2,
                            WIC_PALETTE_ICEFIRE,
                            WIC_PALETTE_RAIN,
                            WIC_PALETTE_REDHOT,
                            WIC_PALETTE_GREENHOT))
        .defaultValue(WIC_PALETTE_WHITE_HOT)
        .description("Camera color palette")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param("Camera Brightness", m_args.camera_brightness)
        .minimumValue("0")
        .maximumValue("16383")
        .defaultValue("8192")
        .description("Camera brightness value (needed for AGC types 'Manual' and 'Auto-Bright')")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param("Camera AGC Type", m_args.camera_agc_type)
        .values(String::str("%s,%s,%s,%s,%s,%s",
                            WIC_AGC_PLATEAU_HISTOGRAM,
                            WIC_AGC_ONCE_BRIGHT,
                            WIC_AGC_AUTO_BRIGHT,
                            WIC_AGC_MANUAL,
                            WIC_AGC_NOT_DEFINED,
                            WIC_AGC_LINEAR_AGC))
        .defaultValue(WIC_AGC_MANUAL)
        .description("Camera AGC type")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param("Camera Video Color Mode", m_args.camera_video_color_mode)
        .values(String::str("%s,%s",
                            WIC_VCM_MONOCHROME,
                            WIC_VCM_COLOR))
        .defaultValue(WIC_VCM_COLOR)
        .description("Camera video color mode")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param("Camera Invert Image Horizontally", m_args.camera_invert_image_horizontally)
        .defaultValue("false")
        .description("Inverts the camera image horizontally")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param("Camera Invert Image Vertically", m_args.camera_invert_image_vertically)
        .defaultValue("false")
        .description("Inverts the camera image vertically")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param("Use Hardware Encoding", m_args.use_hw_encoding)
        .defaultValue("true")
        .description("Use NVIDIA hardware-accelerated H264 encoding instead of software encoding")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param("Record Stream to File", m_args.record_to_file)
        .defaultValue("true")
        .description("Also record the video stream to local files on disk")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param("UDP Destination", m_args.udp_destination)
        .defaultValue("127.0.0.1")
        .description("UDP address to which to stream the video")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        bind<IMC::DevDataText>(this);
        bind<IMC::LoggingControl>(this);

        setEntityState(IMC::EntityState::ESTA_BOOT, Code::CODE_INIT);
      }

      void
      onParametersChanged(void)
      {
        if (paramChanged(m_args.camera_range_mode) ||
            paramChanged(m_args.camera_ffc_mode) ||
            paramChanged(m_args.camera_digital_output_depth) ||
            paramChanged(m_args.camera_palette) ||
            paramChanged(m_args.camera_brightness) ||
            paramChanged(m_args.camera_agc_type) ||
            paramChanged(m_args.camera_video_color_mode) ||
            paramChanged(m_args.camera_invert_image_horizontally) ||
            paramChanged(m_args.camera_invert_image_vertically))
        {
          updateCameraSettings();
        }

        if (paramChanged(m_args.use_hw_encoding) ||
            paramChanged(m_args.record_to_file) ||
            paramChanged(m_args.udp_destination))
        {
          updatePipelineSettings();
        }
      }

      void
      onResourceAcquisition(void)
      {
        connectToCamera();
      }

      void
      onResourceInitialization(void)
      {
        m_camera_if.getCameraSettings();
        updateCameraSettings();
        m_camera_if.printCameraInformation();

        m_pipeline.initialize();
        updatePipelineSettings();
      }

      void
      onResourceRelease(void)
      {
        releasePipeline();
        stopCameraImageAcquisition();
        disconnectFromCamera();
      }

      void
      consume(const IMC::DevDataText* msg)
      {
        // TODO: Check destination ID
        spew("consumed DevDataText message");
        if (msg->value == "TakePicture")
          m_take_picture = true;
      }

      void
      consume(const IMC::LoggingControl* msg)
      {
        spew("consumed LoggingControl message");
        if (m_args.record_to_file)
        {
          switch (msg->op)
          {
            case IMC::LoggingControl::COP_CURRENT_NAME:
            case IMC::LoggingControl::COP_STARTED:
            {
              const Path save_location = m_ctx.dir_log/msg->name;
              if (save_location.type() != Path::Type::PT_DIRECTORY)
              {
                err("not a directory: '%s' - not updating save location", save_location.str().c_str());
              }
              else if (save_location != m_pipeline.saveLocation())
              {
                m_pipeline.setSaveLocation(save_location);
                m_restart_pipeline = true;
              }
              break;
            }
            default:
              break;
          }
        }
      }

      void
      releasePipeline()
      {
        if (!m_pipeline.stop())
          war("%s", m_pipeline.error().c_str());
        m_pipeline.cleanup();
      }

      void
      restartPipelineIfRequested()
      {
        if (!m_restart_pipeline)
          return;

        m_restart_pipeline = false;

        inf("(re)starting pipeline");
        m_pipeline_restart_timer.reset();

        releasePipeline();

        if (!m_pipeline.createAndStart())
        {
          err("%s", m_pipeline.error().c_str());
          return;
        }

        inf("pipeline restarted");
      }

      void
      updatePipelineSettings(void)
      {
        PipelineSettings settings;
        settings.use_hw_encoding = m_args.use_hw_encoding;
        settings.record_to_file = m_args.record_to_file;
        settings.source_frame_width = m_camera_if.cameraResolutionX();
        settings.source_frame_height = m_camera_if.cameraResolutionY();
        settings.source_framerate = m_camera_if.cameraSpeed();
        settings.udp_destination = m_args.udp_destination;

        m_pipeline.updateSettings(settings);
        m_restart_pipeline = true;
      }

      void
      takePictureIfRequested()
      {
        if (!m_take_picture)
          return;

        m_take_picture = false;

        spew("taking picture");

        // TODO
      }

      void
      connectToCamera(void)
      {
        setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_CONNECTING);

        if (!m_camera_if.connect(m_args.camera_license_file_location))
        {
          err("%s - restarting", m_camera_if.error().c_str());
          throw RestartNeeded(m_camera_if.error(), c_restart_delay);
        }

        inf("connected to WIC");
      }

      void
      updateCameraSettings(void)
      {
        CameraSettings settings;
        settings.range = toRange(m_args.camera_range_mode);
        settings.ffc = toFFC(m_args.camera_ffc_mode);
        settings.dod = toDOD(m_args.camera_digital_output_depth);
        settings.palette = toPalette(m_args.camera_palette);
        settings.brightness = m_args.camera_brightness;
        settings.agc = toAGC(m_args.camera_agc_type);
        settings.vcm = toVCM(m_args.camera_video_color_mode);
        settings.invert_hor = m_args.camera_invert_image_horizontally;
        settings.invert_ver = m_args.camera_invert_image_vertically;

        if (!m_camera_if.updateSettings(settings))
          err("%s", m_camera_if.error().c_str());
      }

      void
      startCameraImageAcquisition()
      {
        if (!m_camera_if.startImageAcquisition())
        {
          err("%s - restarting", m_camera_if.error().c_str());
          throw RestartNeeded(m_camera_if.error(), c_restart_delay);
        }
      }

      void
      stopCameraImageAcquisition()
      {
        if(!m_camera_if.stopImageAcquisition())
          war("%s", m_camera_if.error().c_str());
      }

      void
      disconnectFromCamera(void)
      {
        if (!m_camera_if.disconnect())
          war("%s", m_camera_if.error().c_str());
        else
          inf("disconnected from WIC");
      }

      void
      updateTaskErrorsAndState()
      {
        std::vector<std::string> status_errs;

        if (!m_camera_if.hasData())
        {
          status_errs.emplace_back("no camera data");
          m_errors.set(Error::NO_CAMERA_DATA, "no data from camera!");
        }
        else
          m_errors.unset(Error::NO_CAMERA_DATA, "getting camera data");

        if (m_pipeline.state() == PipelineState::ERROR)
        {
          status_errs.emplace_back("pipeline error");
          m_errors.set(Error::PIPELINE_ERROR, "pipeline is in error state!");
        }
        else
          m_errors.unset(Error::PIPELINE_ERROR);

        if (m_pipeline.state() == PipelineState::NOT_PLAYING && m_pipeline_restart_timer.overflow())
        {
          status_errs.emplace_back("pipeline timeout");
          m_errors.set(Error::PIPELINE_TIMEOUT, "pipeline did not start within timeout!");
        }
        else
          m_errors.unset(Error::PIPELINE_TIMEOUT);

        if (!status_errs.empty())
          setEntityState(IMC::EntityState::ESTA_ERROR, String::join(status_errs.begin(), status_errs.end(), ", "));
        else
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);

        for (std::string& msg : m_errors.setMsgs())
          err("%s", msg.c_str());
        for (std::string& msg : m_errors.unsetMsgs())
          inf("%s", msg.c_str());
      }

      void
      onMain(void)
      {
        startCameraImageAcquisition();

        while (!stopping())
        {
          waitForMessages(0.03);

          restartPipelineIfRequested();
          takePictureIfRequested();
          m_pipeline.monitorBus();
          m_pipeline.pushData(m_camera_if.getFrame());
          m_camera_if.releaseFrame();
          updateTaskErrorsAndState();
        }
      }
    };
  }
}

DUNE_TASK
