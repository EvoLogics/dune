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

// C++ STL headers.
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Library headers.
#include <Camera.h>
#include <CameraCenter.h>
#include <CameraSerialSettings.h>

// Local headers.
#include "Constants.hpp"

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    // Convenience typedef.
    typedef CameraSerialSettings CSS;

    // Struct holding the camera settings.
    struct CameraSettings
    {
      //! Whether or not to invert the image horizontally.
      bool invert_hor;
      //! Whether or not to invert the image vertically.
      bool invert_ver;
      //! The scene emissivity (used by camera for temperature calculations) (range: 0.5 - 1.0).
      double emissivity;
      //! The humidity (used by camera for temperature calculations) (range: 0.0 - 1.0).
      double humidity;
      //! The scene atmospheric temperature (used by camera for temperature calculations).
      double atmospheric_temperature;
      //! The scene reflected temperature (used by camera for temperature calculations).
      double reflected_temperature;
    };

    inline std::string
    toStr(const CSS::CameraSpeed fps)
    {
      switch (fps)
      {
        case CSS::CameraSpeed::_9Hz:
          return "9 Hz";
        case CSS::CameraSpeed::_30Hz:
          return "30 Hz";
        case CSS::CameraSpeed::_60Hz:
          return "60 Hz";
        default:
          return "(unrecognized value)";
      }
    }

    inline int
    toInt(const CSS::CameraSpeed fps)
    {
      switch (fps)
      {
        case CSS::CameraSpeed::_9Hz:
          return 9;
        case CSS::CameraSpeed::_30Hz:
          return 30;
        case CSS::CameraSpeed::_60Hz:
          return 60;
        default:
          return 0;
      }
    }

    inline std::string
    toStr(const CSS::DigitalOutputModes dom)
    {
      switch (dom)
      {
        case CSS::DigitalOutputModes::NONE:
          return "None";
        case CSS::DigitalOutputModes::XPMode:
          return "XP Mode";
        case CSS::DigitalOutputModes::LVDSMode:
          return "LVDS Mode";
        case CSS::DigitalOutputModes::CMOSBitDepth:
          return "CMOS Bit Depth";
        case CSS::DigitalOutputModes::LVDSBitDepth:
          return "LVDS Bit Depth";
        default:
          return "(unrecognized value)";
      }
    }

    inline std::string
    toStr(const CSS::RangeModes range)
    {
      switch (range)
      {
        case CSS::RangeModes::Low:
          return "Low";
        case CSS::RangeModes::Middle:
          return "Middle";
        case CSS::RangeModes::High:
          return "High";
        default:
          return "(unrecognized value)";
      }
    }

    inline std::string
    toStr(const CSS::FFCModes ffc)
    {
      switch (ffc)
      {
        case CSS::FFCModes::Manual:
          return "Manual";
        case CSS::FFCModes::Auto:
          return "Auto";
        case CSS::FFCModes::External:
          return "External";
        default:
          return "(unrecognized value)";
      }
    }

    inline std::string
    toStr(const CSS::XPBusModes xp)
    {
      switch (xp)
      {
        case CSS::XPBusModes::Disabled:
          return "Disabled";
        case CSS::XPBusModes::BT656:
          return "BT656";
        case CSS::XPBusModes::CMOS:
          return "CMOS";
        default:
          return "(unrecognized value)";
      }
    }

    inline std::string
    toStr(const CSS::DigitalOutputDepth dod)
    {
      switch (dod)
      {
        case CSS::DigitalOutputDepth::Bits8b:
          return "8 Bit Grayscale";
        case CSS::DigitalOutputDepth::Bit8bBayer:
          return "8 Bit Bayer";
        case CSS::DigitalOutputDepth::Bits14b:
          return "14 Bit Raw";
        case CSS::DigitalOutputDepth::Bit16bYCbCr:
          return "16 Bit YCbCr";
        default:
          return "(unrecognized value)";
      }
    }

    inline std::string
    toStr(const bool value)
    {
      return value ? "true" : "false";
    }

    inline std::string
    toStr(const double value)
    {
      return String::str("%.3f", value);
    }

    class CameraInterface
    {
    public:
      CameraInterface(Task* parent):
        m_camera {nullptr},
        m_has_data {true},
        m_parent {parent}
      {
      }

      ~CameraInterface()
      {
        deallocate();
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
      connect(std::string& camera_license_file_dir)
      {
        if (m_camera == nullptr)
        {
          if (!findCamera(camera_license_file_dir))
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
          m_parent->debug("camera not connected - not stopping image acquisition");
          return true;
        }

        m_camera->StopAcquisition();

        if (m_camera->IsAcquiring())
        {
          m_error = "failed to stop camera acquisition!";
          return false;
        }

        resetError();
        return true;
      }

      double
      tempConversionSlope()
      {
        return m_temp_conversion_slope;
      }

      double
      tempConversionOffset()
      {
        return m_temp_conversion_offset;
      }

      bool
      updateSettings(const CameraSettings& settings)
      {
        if (m_camera == nullptr)
        {
          m_parent->debug("camera not connected - not updating settings");
          return true;
        }

        std::vector<std::string> error_msgs;
        auto add_to_errors = [&error_msgs](const std::string& param_name, const std::string& value)
        {
          error_msgs.push_back("'" + param_name + "' to '" + value + "'");
        };

        CSS* css = m_camera->GetSettings();
        std::function<void(bool)> invert_hor_setter = std::bind(&CSS::SetRevertVideo, css, std::placeholders::_1);
        std::function<bool(void)> invert_hor_getter = std::bind(&CSS::GetRevertVideo, css);
        std::function<void(bool)> invert_ver_setter = std::bind(&CSS::SetInvertVideo, css, std::placeholders::_1);
        std::function<bool(void)> invert_ver_getter = std::bind(&CSS::GetInvertVideo, css);
        std::function<void(double)> emissivity_setter = std::bind(&CSS::SetEmissivity, css, std::placeholders::_1);
        std::function<double(void)> emissivity_getter = std::bind(&CSS::GetEmissivity, css);
        std::function<void(double)> humidity_setter = std::bind(&CSS::SetHumidity, css, std::placeholders::_1);
        std::function<double(void)> humidity_getter = std::bind(&CSS::GetHumidity, css);
        std::function<void(double)> atmospheric_T_setter = std::bind(&CSS::SetAtmospericTemperatureC, css, std::placeholders::_1);
        std::function<double(void)> atmospheric_T_getter = std::bind(&CSS::GetAtmospericTemperatureC, css);
        std::function<void(double)> reflected_T_setter = std::bind(&CSS::SetReflectedTemperatureC, css, std::placeholders::_1);
        std::function<double(void)> reflected_T_getter = std::bind(&CSS::GetReflectedTemperatureC, css);

        if (!updateSetting("invert image horizontally", settings.invert_hor, m_settings.invert_hor,
                           invert_hor_setter, invert_hor_getter))
          add_to_errors("invert image horizontally", toStr(settings.invert_hor));
        if (!updateSetting("invert image vertically", settings.invert_ver, m_settings.invert_ver,
                           invert_ver_setter, invert_ver_getter))
          add_to_errors("invert image vertically", toStr(settings.invert_ver));
        if (!updateSetting("humidity", settings.humidity, m_settings.humidity,
                           humidity_setter, humidity_getter))
          add_to_errors("humidity", toStr(settings.humidity));
        if (!updateSetting("emissivity", settings.emissivity, m_settings.emissivity,
                           emissivity_setter, emissivity_getter))
          add_to_errors("emissivity", toStr(settings.emissivity));
        if (!updateSetting("atmospheric temperature", settings.atmospheric_temperature, m_settings.atmospheric_temperature,
                           atmospheric_T_setter, atmospheric_T_getter))
          add_to_errors("atmospheric temperature", toStr(settings.atmospheric_temperature));
        if (!updateSetting("reflected temperature", settings.reflected_temperature, m_settings.reflected_temperature,
                           reflected_T_setter, reflected_T_getter))
          add_to_errors("reflected temperature", toStr(settings.reflected_temperature));

        if (error_msgs.size() > 0)
        {
          m_error = String::str("failed to update camera setting(s): %s",
                                String::join(error_msgs.begin(), error_msgs.end(), ", ").c_str());
          return false;
        }

        //! Calculate the slope and offset for temperature conversions.
        const double temp_min {m_camera->CalculateTemperatureC(0U)};
        const double temp_max {m_camera->CalculateTemperatureC(7000U)};
        m_temp_conversion_offset = temp_min;
        m_temp_conversion_slope = (temp_max - temp_min) / 7000U;

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

        const std::string manufacturer = css->GetManufacturer();
        const std::string model = css->GetModel();
        const std::string serial_nb = String::str("%d", css->GetCameraSerialNumber());
        const std::string fw_str = String::str("%d-%d", css->GetFWMajorVersion(), css->GetFWMinorVersion());
        const std::string sw_str = String::str("%d-%d", css->GetSWMajorVersion(), css->GetSWMinorVersion());
        const std::string res_str = String::str("%dx%d", css->GetResolutionX(), css->GetResolutionY());
        const std::string speed_str = toStr(css->GetCameraSpeed());
        const std::string range_str = toStr(css->GetRangeMode());
        const std::string ffc_str = toStr(css->GetFFCMode());
        const std::string xpbusmode_str = toStr(css->GetXPBusMode());
        const std::string dod_str = toStr(css->GetCMOSBitDepth());
        const std::string emissivity_str = String::str("%.2f", css->GetEmissivity());
        const std::string humidity_str = String::str("%.2f", css->GetHumidity());
        const std::string atmospheric_temp_str = String::str("%.2f", css->GetAtmospericTemperatureC());
        const std::string reflected_temp_str = String::str("%.2f", css->GetReflectedTemperatureC());

        auto print_camera_setting = [this](const std::string& name, const std::string value)
        {
          m_parent->debug("%-21s: %s", name.c_str(), value.c_str());
        };

        m_parent->debug("======= CAMERA INFORMATION =======");
        print_camera_setting("Manufacturer", manufacturer);
        print_camera_setting("Model", model);
        print_camera_setting("Serial Number", serial_nb);
        print_camera_setting("Firmware Version", fw_str);
        print_camera_setting("Software Version", sw_str);
        print_camera_setting("Resolution", res_str);
        print_camera_setting("Speed", speed_str);
        print_camera_setting("Range Mode", range_str);
        print_camera_setting("FFC Mode", ffc_str);
        print_camera_setting("XP Bus Mode", xpbusmode_str);
        print_camera_setting("Digital Output Depth", dod_str);
        print_camera_setting("Emissivity", emissivity_str);
        print_camera_setting("Humidity", humidity_str);
        print_camera_setting("Atmospheric Temp", atmospheric_temp_str);
        print_camera_setting("Reflected Temp", reflected_temp_str);
        m_parent->debug("==================================");
      }

    private:
      bool
      findCamera(const std::string& camera_license_file_dir)
      {
        deallocate();
        m_camera_center = new CameraCenter(camera_license_file_dir);
        if (m_camera_center->getCameras().empty())
          return false;

        // We always get the first camera in the list, assuming it is our camera.
        // This works in our system because we only have 1 camera connected.
        // Maybe this logic could be improved by checking the camera type/model/... before assigning to m_camera?
        m_camera = m_camera_center->getCameras().front();
        return true;
      }

      void
      deallocate()
      {
        if (m_camera_center)
          delete m_camera_center;
      }

      void
      checkCameraSettingsNotNull(CSS* css)
      {
        if (css == nullptr)
          throw RestartNeeded(DTR("camera returned NULL when settings were queried"), c_restart_delay);
      }

      template <typename T>
      bool
      updateSetting(const std::string& name,
                    const T value,
                    T& current_value,
                    std::function<void(T)> setter,
                    std::function<T(void)> getter)
      {
        if (value == current_value)
        {
          m_parent->debug("camera setting '%s' already set to '%s' - ignoring", name.c_str(), toStr(value).c_str());
          return true;
        }

        m_parent->inf("updating camera setting '%s' to '%s'", name.c_str(), toStr(value).c_str());
        setter(value);
        if (getter() != value)
          return false;

        current_value = value;
        return true;
      }

      void
      resetError()
      {
        m_error.clear();
      }

      CameraCenter* m_camera_center;
      Camera* m_camera;
      CameraSettings m_settings;
      std::string m_error;
      double m_temp_conversion_slope;
      double m_temp_conversion_offset;
      bool m_has_data;
      Task* m_parent;
    };
  }
}

#endif
