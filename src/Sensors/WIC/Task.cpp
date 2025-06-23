//***************************************************************************
// Copyright 2007-2024 EvoLogics GmbH                                       *
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

// C++ STL headers.
#include <string>
#include <vector>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include "Constants.hpp"
#include "CameraInterface.hpp"
#include "ImageProcessor.hpp"
#include "PaletteLoader.hpp"
#include "Pipeline.hpp"

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    //! Enum representing Task error states.
    enum Error
    {
      //! No data received from the camera.
      NO_CAMERA_DATA,
      //! The GStreamer pipeline is in error.
      PIPELINE_ERROR,
      //! The GStreamer pipeline timed out.
      PIPELINE_TIMEOUT,
    };

    //! Task arguments.
    struct Arguments
    {
      //! Path to the directory containing the WIC license file in the file system.
      std::string wic_license_file_location;
      //! If true, invert the image horizontally.
      bool invert_image_horizontally;
      //! If true, invert the image vertically.
      bool invert_image_vertically;
      //! Emissitivity value of the camera (used in temperature calculations).
      double camera_emissivity;
      //! Humidity value of the camera (used in temperature calculations).
      double camera_humidity;
      //! Atmospheric temperature value of the camera (used in temperature calculations).
      double camera_atmospheric_temperature;
      //! Reflected temperature value of the camera (used in temperature calculations).
      double camera_reflected_temperature;
      //! Whether to use HW or SW encoding for streaming & recording the video.
      bool use_hw_encoding;
      //! Whether to record the video stream.
      bool record_to_file;
      //! The path in the filetree where to save recordings and pictures.
      std::string recordings_and_pictures_path;
      //! IPv4 address to which to stream the UDP video stream.
      std::string udp_destination;
      //! Color palette to apply to the camera values.
      std::string color_palette;
      //! Color scaling method to use.
      std::string color_scaling_method;
      //! Amount of standard deviations from the frame mean value to color grade before saturation.
      double auto_scaling_std_dev_factor;
      //! In case of manual color scaling, min temperature as from which to apply the colorscale.
      double manual_color_scaling_min_temperature;
      //! In case of manual color scaling, max temperature up to which to apply the colorscale.
      double manual_color_scaling_max_temperature;
      //! If true, invert the colors of the color palette.
      bool invert_colors;
      //! Method to use for contrast correction.
      std::string contrast_correction_method;
      //! In case of gamma contrast correction, the exponen to use.
      double gamma_correction_factor;
      //! In case of histogram equilization contrast correction, the tile size to use.
      int clahe_tile_size;
      //! In case of histogram equilization contrast correction, the clip limit to use.
      double clahe_clip_limit;
      //! The framerate at which to stream the video.
      int stream_framerate;
      //! Whether to display the color scale overlay.
      bool display_color_scale_overlay;
    };

    struct Task: public DUNE::Tasks::Task
    {
      //! Task arguments.
      Arguments m_args;
      //! Camera interface (to query and control to camera).
      CameraInterface m_camera_if;
      //! Error aggregator (keep track of Task errors).
      ErrorAggregator m_errors;
      //! Wrapper around the GStreamer pipeline.
      Pipeline m_pipeline;
      //! Utility class to load the color palettes.
      PaletteLoader m_palette_loader;
      //! Processor to convert the raw camera data to an image (add color, apply contrast correction, ...).
      ImageProcessor m_image_processor;
      //! Whether to restart the pipeline.
      bool m_restart_pipeline;
      //! Whether to save the current frame as a picture.
      bool m_take_picture;
      //! Timer to check the pipeline has not timed out (= has not become operational within a certain period).
      Counter<double> m_pipeline_timeout_timer;

      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_camera_if {this},
        m_errors {this},
        m_pipeline {this},
        m_palette_loader {ctx.dir_cfg / c_color_palette_dir, this},
        m_image_processor {this},
        m_restart_pipeline {false},
        m_take_picture {false},
        m_pipeline_timeout_timer {c_pipeline_start_timeout}
      {
        setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_INIT);

        setStackSize(c_thread_stack_size);
        inf("increased Task stack size to: %lu", getStackSize());

        std::vector<std::string> all_palettes = m_palette_loader.paletteNames();
        param(DTR_RT("Color Palette"), m_args.color_palette)
        .values(String::join(all_palettes.begin(), all_palettes.end(), ","))
        .defaultValue(all_palettes.front())
        .description("Color palette to apply to the image. "
                     "The color palette files are read from disk. "
                     "The default 'None' is always available and applies a grayscale palette.")
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("Camera - Emissivity"), m_args.camera_emissivity)
        .defaultValue("0.95")
        .minimumValue("0.5")
        .maximumValue("1.0")
        .description("Emissivity setting of the camera (used by the camera to calculate temperatures).")
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param(DTR_RT("Camera - Humidity"), m_args.camera_humidity)
        .defaultValue("0.05")
        .minimumValue("0.0")
        .maximumValue("1.0")
        .description("Humidity setting of the camera (used by the camera to calculate temperatures).")
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param(DTR_RT("Camera - Atmospheric Temperature"), m_args.camera_atmospheric_temperature)
        .defaultValue("20")
        .units(Units::DegreeCelsius)
        .description("Atmospheric temperature setting of the camera (used by the camera to calculate temperatures).")
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param(DTR_RT("Camera - Reflected Temperature"), m_args.camera_reflected_temperature)
        .defaultValue("20")
        .units(Units::DegreeCelsius)
        .description("Reflected temperature setting of the camera (used by the camera to calculate temperatures).")
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param(DTR_RT("Color Scaling - Method"), m_args.color_scaling_method)
        .values(String::str("%s,%s,%s",
                            c_color_scaling_method_full_scale.c_str(),
                            c_color_scaling_method_smoothed.c_str(),
                            c_color_scaling_method_manual.c_str()))
        .defaultValue(c_color_scaling_method_smoothed)
        .description("The color scaling method to apply.")
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("Color Scaling - Std Dev Factor"), m_args.auto_scaling_std_dev_factor)
        .defaultValue("2.5")
        .minimumValue("0.001")
        .description("Amount of standard deviations around the mean to consider for coloring the image. "
                     "Only valid for mode 'Auto Std Dev'.")
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("Color Scaling - Manual Min Temperature"), m_args.manual_color_scaling_min_temperature)
        .defaultValue("0.0")
        .description("Minimum temperature from which to start coloring the image. "
                     "Only valid for mode 'Manual'.")
        .units(Units::DegreeCelsius)
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("Color Scaling - Manual Max Temperature"), m_args.manual_color_scaling_max_temperature)
        .defaultValue("30.0")
        .description("Maximum temperature until which to color the image. "
                     "Only valid for mode 'Manual'.")
        .units(Units::DegreeCelsius)
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("Invert Colors"), m_args.invert_colors)
        .defaultValue("false")
        .description("Invert colors in the color current palette.")
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("Record Video"), m_args.record_to_file)
        .defaultValue("true")
        .description("Record the video stream as a .avi file.")
        .scope(Tasks::Parameter::SCOPE_MANEUVER)
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("Recordings And Pictures Path"), m_args.recordings_and_pictures_path)
        .defaultValue("/mnt/storage/wic/")
        .description("Top-level path where to store the recording and the pictures.")
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param(DTR_RT("Contrast Correction Method"), m_args.contrast_correction_method)
        .values(String::str("%s,%s,%s",
                            c_contrast_correction_none.c_str(),
                            c_contrast_correction_gamma.c_str(),
                            c_contrast_correction_clahe.c_str()))
        .defaultValue(c_contrast_correction_none)
        .description("Method to use for contrast correction.")
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("Gamma Correction"), m_args.gamma_correction_factor)
        .defaultValue("2.0")
        .minimumValue("0.0")
        .description("Exponent used for the gamma correction.")
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("CLAHE Tile Size"), m_args.clahe_tile_size)
        .defaultValue("16")
        .minimumValue("1")
        .description("CLAHE tile size, in pixels.")
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("CLAHE Clip Limit"), m_args.clahe_clip_limit)
        .defaultValue("5.0")
        .minimumValue("0.0")
        .description("CLAHE clip limit.")
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("Invert Image Horizontally"), m_args.invert_image_horizontally)
        .defaultValue("false")
        .description("Invert the camera image horizontally")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param(DTR_RT("Invert Image Vertically"), m_args.invert_image_vertically)
        .defaultValue("false")
        .description("Invert the camera image vertically")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        param(DTR_RT("WIC License File Location"), m_args.wic_license_file_location)
        .defaultValue("/")
        .description("Absolute path of the WIC camera license file")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER);

        /* param(DTR_RT("Use Hardware Encoding"), m_args.use_hw_encoding) */
        /* .defaultValue("true") */
        /* .description("Use NVIDIA hardware-accelerated H264 encoding instead of software encoding.") */
        /* .units(Units::None) */
        /* .visibility(Tasks::Parameter::VISIBILITY_DEVELOPER); */
        m_args.use_hw_encoding = true;

        param(DTR_RT("UDP Destination"), m_args.udp_destination)
        .defaultValue("127.0.0.1")
        .description("UDP address to which to stream the video.")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("Stream Framerate"), m_args.stream_framerate)
        .defaultValue("10")
        .minimumValue("1")
        .maximumValue("30")
        .description("Framerate at which to stream the video.")
        .units(Units::Hertz)
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        param(DTR_RT("Display Color Scale"), m_args.display_color_scale_overlay)
        .defaultValue("false")
        .description("Whether to display the color scale overlay.")
        .units(Units::None)
        .visibility(Tasks::Parameter::VISIBILITY_USER);

        bind<IMC::LoggingControl>(this);
      }

      void
      onParametersChanged(void)
      {
        if (paramChanged(m_args.invert_image_horizontally) ||
            paramChanged(m_args.invert_image_vertically) ||
            paramChanged(m_args.camera_emissivity) ||
            paramChanged(m_args.camera_humidity) ||
            paramChanged(m_args.camera_atmospheric_temperature) ||
            paramChanged(m_args.camera_reflected_temperature))
        {
          updateCameraSettings();
        }

        if (paramChanged(m_args.record_to_file) ||
            paramChanged(m_args.udp_destination) ||
            paramChanged(m_args.stream_framerate))
        {
          updatePipelineSettings();
        }

        if (paramChanged(m_args.color_palette) ||
            paramChanged(m_args.color_scaling_method) ||
            paramChanged(m_args.auto_scaling_std_dev_factor) ||
            paramChanged(m_args.manual_color_scaling_min_temperature) ||
            paramChanged(m_args.manual_color_scaling_max_temperature) ||
            paramChanged(m_args.invert_colors))
        {
          updateColorSettings();
        }

        if (paramChanged(m_args.contrast_correction_method) ||
            paramChanged(m_args.gamma_correction_factor) ||
            paramChanged(m_args.clahe_tile_size) ||
            paramChanged(m_args.clahe_clip_limit))
        {
          updateContrastCorrectionSettings();
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
        updateCameraSettings();
        m_camera_if.printCameraInformation();

        m_pipeline.initialize();
        updatePipelineSettings();

        m_palette_loader.loadPalette(m_args.color_palette);
        updateColorSettings();
        updateContrastCorrectionSettings();

        startCameraImageAcquisition();
      }

      void
      onResourceRelease(void)
      {
        releasePipeline();
        stopCameraImageAcquisition();
        disconnectFromCamera();
      }

      void
      consume(const IMC::LoggingControl* msg)
      {
        switch (msg->op)
        {
          case IMC::LoggingControl::COP_STARTED:
            debug("received LoggingControl::COP_START - setting location for recordings and restarting pipeline");
            setLocationForRecordingsAndPictures(msg->name);
            m_restart_pipeline = true;
            break;
          default:
            break;
        }
      }

      void
      setLocationForRecordingsAndPictures(const std::string& path)
      {
        const FileSystem::Path save_directory {m_args.recordings_and_pictures_path / path};

        if (save_directory.type() != FileSystem::Path::Type::PT_DIRECTORY)
        {
          debug("save location for videos/pictures does not exist: '%s' - creating", save_directory.str().c_str());
          try
          {
            save_directory.create();
            m_pipeline.setRecordDirectory(save_directory);
            m_image_processor.setPictureDirectory(save_directory);
          }
          catch (const std::exception& e)
          {
            war("failed to create directory for saving recordings and pictures: %s", e.what());
          }
        }
      }

      void
      updateCameraSettings()
      {
        CameraSettings settings;
        settings.invert_hor = m_args.invert_image_horizontally;
        settings.invert_ver = m_args.invert_image_vertically;
        settings.emissivity = m_args.camera_emissivity;
        settings.humidity = m_args.camera_humidity;
        settings.atmospheric_temperature = m_args.camera_atmospheric_temperature;
        settings.reflected_temperature = m_args.camera_reflected_temperature;

        if (!m_camera_if.updateSettings(settings))
          err("%s", m_camera_if.error().c_str());
      }

      void
      updateColorSettings()
      {
        m_palette_loader.loadPalette(m_args.color_palette);

        ColorSettings settings;
        settings.palette = m_palette_loader.palette();
        settings.color_scaling_method = colorScalingFromStr(m_args.color_scaling_method);
        settings.auto_color_scaling_std_dev_factor = m_args.auto_scaling_std_dev_factor;
        settings.manual_color_scaling_min_temperature = m_args.manual_color_scaling_min_temperature;
        settings.manual_color_scaling_max_temperature = m_args.manual_color_scaling_max_temperature;
        settings.invert_colors = m_args.invert_colors;
        settings.raw_to_temp_slope = m_camera_if.tempConversionSlope();
        settings.raw_to_temp_offset = m_camera_if.tempConversionOffset();

        m_image_processor.updateColorSettings(settings);
      }

      void
      updateContrastCorrectionSettings()
      {
        ContrastCorrectionSettings settings;
        settings.method = contrastFromStr(m_args.contrast_correction_method);
        settings.gamma_correction_factor = m_args.gamma_correction_factor;
        settings.clahe_tile_size = m_args.clahe_tile_size;
        settings.clahe_clip_limit = m_args.clahe_clip_limit;

        m_image_processor.updateContrastCorrectionSettings(settings);
      }

      void
      updatePipelineSettings()
      {
        PipelineSettings settings;
        settings.use_hw_encoding = m_args.use_hw_encoding;
        settings.record_to_file = m_args.record_to_file;
        settings.frame_width = m_camera_if.cameraResolutionX();
        settings.frame_height = m_camera_if.cameraResolutionY();
        settings.src_framerate = m_camera_if.cameraSpeed();
        settings.stream_framerate = m_args.stream_framerate;
        settings.udp_destination = m_args.udp_destination;

        m_pipeline.updateSettings(settings);
        m_restart_pipeline = true;

        m_image_processor.init(settings.frame_width, settings.frame_height);
      }

      void
      connectToCamera()
      {
        setEntityState(IMC::EntityState::ESTA_BOOT, Status::CODE_CONNECTING);

        if (!m_camera_if.connect(m_args.wic_license_file_location))
          throw RestartNeeded(m_camera_if.error(), c_restart_delay);

        inf("connected to WIC");
      }

      void
      startCameraImageAcquisition()
      {
        if (!m_camera_if.startImageAcquisition())
          throw RestartNeeded(m_camera_if.error(), c_restart_delay);
      }

      void
      stopCameraImageAcquisition()
      {
        if(!m_camera_if.stopImageAcquisition())
          war("%s", m_camera_if.error().c_str());
      }

      void
      disconnectFromCamera()
      {
        if (!m_camera_if.disconnect())
          war("%s", m_camera_if.error().c_str());
        else
          inf("disconnected from WIC");
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

        debug("restarting pipeline");

        m_restart_pipeline = false;
        m_pipeline_timeout_timer.reset();

        releasePipeline();

        if (!m_pipeline.create())
        {
          err("%s", m_pipeline.error().c_str());
          return;
        }

        if (!m_pipeline.start())
        {
          err("%s", m_pipeline.error().c_str());
          return;
        }
      }

      void
      takePictureIfRequested()
      {
        if (!m_take_picture)
          return;

        m_take_picture = false;
        spew("taking picture");
        m_image_processor.saveProcessedImage();
      }

      void
      updateTaskErrorsAndState()
      {
        std::vector<std::string> status_errs;

        if (!m_camera_if.hasData())
        {
          status_errs.emplace_back("no camera data");
          m_errors.setError(Error::NO_CAMERA_DATA, "no data from camera!");
        }
        else
          m_errors.unsetError(Error::NO_CAMERA_DATA);

        if (m_pipeline.state() == PipelineState::ERROR)
        {
          status_errs.emplace_back("pipeline error");
          m_errors.setError(Error::PIPELINE_ERROR, "pipeline is in error state!");
        }
        else
          m_errors.unsetError(Error::PIPELINE_ERROR);

        if (m_pipeline.state() == PipelineState::NOT_PLAYING && m_pipeline_timeout_timer.overflow())
        {
          status_errs.emplace_back("pipeline timeout");
          m_errors.setError(Error::PIPELINE_TIMEOUT,
                            String::str("pipeline did not start within %.2f s", c_pipeline_start_timeout));
        }
        else
          m_errors.unsetError(Error::PIPELINE_TIMEOUT);

        if (!status_errs.empty())
          setEntityState(IMC::EntityState::ESTA_ERROR, String::join(status_errs.begin(), status_errs.end(), ", "));
        else
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      void
      onMain(void)
      {
        while (!stopping())
        {
          // 0.03 seconds is smaller period than data rate from camera (30 Hz).
          // This value could be made dependent on the camera rate (which we can get from camera settings).
          waitForMessages(0.03);

          restartPipelineIfRequested();
          takePictureIfRequested();
          m_image_processor.processRawImage(m_camera_if.getFrame(), m_args.display_color_scale_overlay);
          m_pipeline.monitorBus();
          m_pipeline.pushData(m_image_processor.processedImage());
          m_camera_if.releaseFrame();
          updateTaskErrorsAndState();
        }
      }
    };
  }
}

DUNE_TASK
