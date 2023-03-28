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

#ifndef SENSORS_WIC_CAMERA_INTERFACE_HPP_INCLUDED_
#define SENSORS_WIC_CAMERA_INTERFACE_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/DUNE.hpp>

// ISO C++ 11 headers.
#include <functional>
#include <string>
#include <vector>

// Library headers.
#include <Camera.h>
#include <CameraCenter.h>
#include <CameraSerialSettings.h>

// Local headers.
#include "Constants.hpp"
#include "Utils.hpp"

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    struct CameraSettings
    {
      CSS::RangeModes range;
      CSS::FFCModes ffc;
      CSS::DigitalOutputDepth dod;
      CSS::Palettes palette;
      uint16_t brightness;
      CSS::AGCTypes agc;
      CSS::VideoColorModes vcm;
      bool invert_hor;
      bool invert_ver;
    };

    class CameraInterface
    {
    public:
      CameraInterface(Task* parent):
        m_camera {nullptr},
        m_has_data {true},
        m_parent {parent}
      {
      }

      std::string
      error()
      {
        return m_error;
      }

      bool
      hasData()
      {
        return m_has_data;
      }

      bool
      connect(std::string& camera_license_file_folder)
      {
        spew("connecting to camera");

        if (m_camera == nullptr)
        {
          if (!findCamera(camera_license_file_folder))
          {
            m_error = "no camera found";
            return false;
          }
        }

        if (m_camera->Connect() != 0)
        {
          m_error = "failed to connect to camera";
          return false;
        }

        if (!m_camera->IsConnected())
        {
          m_error = "failed to connect to camera";
          return false;
        }

        resetError();
        return true;
      }

      bool
      disconnect()
      {
        spew("disconnecting from camera");

        if (m_camera != nullptr)
        {
          m_camera->Disconnect();
          if (m_camera->IsConnected())
          {
            m_error = "failed to disconnect from camera";
            return false;
          }
        }

        resetError();
        return true;
      }

      bool
      startImageAcquisition()
      {
        spew("starting image acquisition");

        m_camera->StartAcquisition();

        if (!m_camera->IsAcquiring())
        {
          m_error = "failed to start camera acquisition!";
          return false;
        }

        resetError();
        return true;
      }

      bool
      stopImageAcquisition()
      {
        if (m_camera == nullptr)
        {
          spew("camera not connected - not stopping image acquisition");
          return true;
        }

        spew("stopping image acquisition");

        m_camera->StopAcquisition();

        if (m_camera->IsAcquiring())
        {
          m_error = "failed to stop camera acquisition!";
          return false;
        }

        resetError();
        return true;
      }

      bool
      updateSettings(const CameraSettings& settings)
      {
        if (m_camera == nullptr)
        {
          spew("camera not connected - not updating settings");
          return true;
        }

        spew("updating camera settings");

        CSS* css = m_camera->GetSettings();

        std::vector<std::string> error_msgs;

        auto add_to_errors = [&error_msgs](const std::string& param_name, const std::string& value)
        {
          error_msgs.push_back("'" + param_name + "' to '" + value + "'");
        };

        std::function<void(CSS::RangeModes)> range_setter =
          std::bind(&CSS::SetRangeMode, css, std::placeholders::_1);
        std::function<CSS::RangeModes(void)> range_getter =
          std::bind(&CSS::GetRangeMode, css);
        std::function<void(CSS::FFCModes)> ffc_setter =
          std::bind(&CSS::SetFFCMode, css, std::placeholders::_1);
        std::function<CSS::FFCModes(void)> ffc_getter =
          std::bind(&CSS::GetFFCMode, css);
        std::function<void(CSS::DigitalOutputDepth)> dod_setter =
          std::bind(&CSS::SetCMOSBitDepth, css, std::placeholders::_1);
        std::function<CSS::DigitalOutputDepth(void)> dod_getter =
          std::bind(&CSS::GetCMOSBitDepth, css);
        std::function<void(CSS::Palettes)> palette_setter =
          std::bind(&CSS::SetPalette, css, std::placeholders::_1);
        std::function<CSS::Palettes(void)> palette_getter =
          std::bind(&CSS::GetPalette, css);
        std::function<void(uint16_t)> brightness_setter =
          std::bind(&CSS::SetBrightness, css, std::placeholders::_1);
        std::function<uint16_t(void)> brightness_getter =
          std::bind(&CSS::GetBrightness, css);
        std::function<void(CSS::AGCTypes)> agc_setter =
          std::bind(&CSS::SetAGCType, css, std::placeholders::_1);
        std::function<CSS::AGCTypes(void)> agc_getter =
          std::bind(&CSS::GetAGCType, css);
        std::function<void(CSS::VideoColorModes)> vcm_setter =
          std::bind(&CSS::SetVideoColorMode, css, std::placeholders::_1);
        std::function<CSS::VideoColorModes(void)> vcm_getter =
          std::bind(&CSS::GetVideoColorMode, css);
        std::function<void(bool)> invert_hor_setter =
          std::bind(&CSS::SetInvertVideo, css, std::placeholders::_1);
        std::function<bool(void)> invert_hor_getter =
          std::bind(&CSS::GetInvertVideo, css);
        std::function<void(bool)> invert_ver_setter =
          std::bind(&CSS::SetRevertVideo, css, std::placeholders::_1);
        std::function<bool(void)> invert_ver_getter =
          std::bind(&CSS::GetRevertVideo, css);

        if (!updateSetting("range", settings.range, &m_settings.range, range_setter, range_getter))
          add_to_errors("range", toStr(settings.range));
        if (!updateSetting("FFC", settings.ffc, &m_settings.ffc, ffc_setter, ffc_getter))
          add_to_errors("FFC", toStr(settings.ffc));
        if (!updateSetting("digital output depth", settings.dod, &m_settings.dod, dod_setter, dod_getter))
          add_to_errors("digital output depth", toStr(settings.dod));
        if (!updateSetting("palette", settings.palette, &m_settings.palette, palette_setter, palette_getter))
          add_to_errors("palette", toStr(settings.palette));
        if (!updateSetting("brightness", settings.brightness, &m_settings.brightness, brightness_setter, brightness_getter))
          add_to_errors("brightness", toStr(settings.brightness));
        /* if (!updateSetting("AGC", settings.agc, &m_settings.agc, agc_setter, agc_getter)) */
        /*   add_to_errors("AGC", toStr(settings.agc)); */
        if (!updateSetting("video color mode", settings.vcm, &m_settings.vcm, vcm_setter, vcm_getter))
          add_to_errors("video color mode", toStr(settings.vcm));
        if (!updateSetting("invert image horizontally", settings.invert_hor, &m_settings.invert_hor, invert_hor_setter, invert_hor_getter))
          add_to_errors("invert image horizontally", toStr(settings.invert_hor));
        if (!updateSetting("invert image vertically", settings.invert_ver, &m_settings.invert_ver, invert_ver_setter, invert_ver_getter))
          add_to_errors("invert image vertically", toStr(settings.invert_ver));

        if (error_msgs.size() > 0)
        {
          m_error = String::str("failed to update following camera settings: %s",
                                String::join(error_msgs.begin(), error_msgs.end(), ", ").c_str());
          return false;
        }

        resetError();
        return true;
      }

      uint8_t*
      getFrame()
      {
        uint8_t* frame_ptr = m_camera->RetreiveBuffer();
        m_has_data = (frame_ptr != nullptr);
        return frame_ptr;
      }

      void
      releaseFrame()
      {
        m_camera->ReleaseBuffer();
      }

      int
      cameraResolutionX()
      {
        CSS* css = m_camera->GetSettings();
        checkCameraSettingsNotNull(css);
        return css->GetResolutionX();
      }

      int
      cameraResolutionY()
      {
        CSS* css = m_camera->GetSettings();
        checkCameraSettingsNotNull(css);
        return css->GetResolutionY();
      }

      int
      cameraSpeed()
      {
        CSS* css = m_camera->GetSettings();
        checkCameraSettingsNotNull(css);
        return toInt(css->GetCameraSpeed());
      }

      void
      printCameraInformation()
      {
        CSS* css = m_camera->GetSettings();
        checkCameraSettingsNotNull(css);

        const std::string serial_nb = String::str("%d", css->GetCameraSerialNumber());
        const std::string fw_str = String::str("%d-%d", css->GetFWMajorVersion(), css->GetFWMinorVersion());
        const std::string sw_str = String::str("%d-%d", css->GetSWMajorVersion(), css->GetSWMinorVersion());
        const std::string res_str = String::str("%dx%d", css->GetResolutionX(), css->GetResolutionY());
        const std::string speed_str = toStr(css->GetCameraSpeed());
        const std::string range_str = toStr(css->GetRangeMode());
        const std::string ffc_str = toStr(css->GetFFCMode());
        const std::string xpbusmode_str = toStr(css->GetXPBusMode());
        const std::string dod_str = toStr(css->GetCMOSBitDepth());
        const std::string palette_str = toStr(css->GetPalette());
        const std::string brightness_str = toStr(css->GetBrightness());
        const std::string agc_str = toStr(css->GetAGCType());
        const std::string vcm_str = toStr(css->GetVideoColorMode());
        const std::string invert_hor_str = toStr(css->GetInvertVideo());
        const std::string invert_ver_str = toStr(css->GetRevertVideo());

        auto print_camera_setting = [this](const std::string& name, const std::string value)
        {
          this->spew(String::str("%-21s %s", name.c_str(), value.c_str()));
        };

        spew("======= CAMERA INFORMATION =======");
        print_camera_setting("Serial Number:", serial_nb);
        print_camera_setting("Firmware Version:", fw_str);
        print_camera_setting("Software Version:", sw_str);
        print_camera_setting("Resolution:", res_str);
        print_camera_setting("Speed:", speed_str);
        print_camera_setting("Range Mode:", range_str);
        print_camera_setting("FFC Mode:", ffc_str);
        print_camera_setting("XP Bus Mode:", xpbusmode_str);
        print_camera_setting("Digital Output Depth:", dod_str);
        print_camera_setting("Palette:", palette_str);
        print_camera_setting("Brightness:", brightness_str);
        print_camera_setting("AGC Type:", agc_str);
        print_camera_setting("Video Color Mode:", vcm_str);
        print_camera_setting("Invert Image Horiz:", invert_hor_str);
        print_camera_setting("Invert Image Vert:", invert_ver_str);
        spew("==================================");
      }

    private:
      bool
      findCamera(const std::string& camera_license_file_folder)
      {
        CameraCenter *cameras = new CameraCenter(camera_license_file_folder);
        if (cameras->getCameras().empty())
          return false;

        m_camera = cameras->getCameras().front();
        return true;
      }

      void
      checkCameraSettingsNotNull(CSS* css)
      {
        if (css == nullptr)
          throw RestartNeeded(DTR("camera returned NULL when settings were queried"), c_restart_delay);
      }

      template <typename T>
      bool
      updateSetting(const std::string& name, const T value, T* current_value,
                    std::function<void(T)> setter,
                    std::function<T(void)> getter)
      {
        if (value == *current_value)
        {
          spew(String::str("camera setting '%s' already set to '%s' - ignoring", name.c_str(), toStr(value).c_str()));
          return true;
        }

        m_parent->inf("updating camera setting '%s' to '%s'", name.c_str(), toStr(value).c_str());
        setter(value);
        if (getter() != value)
          return false;

        *current_value = value;
        return true;
      }

      void
      resetError()
      {
        m_error.clear();
      }

      void
      spew(const std::string& msg)
      {
        m_parent->spew("[%-8s] %s", "Camera", msg.c_str());
      }

      Camera* m_camera;
      CameraSettings m_settings;
      std::string m_error;
      bool m_has_data;
      Task* m_parent;
    };
  }
}

#endif
