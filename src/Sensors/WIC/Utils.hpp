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

#ifndef SENSORS_WIC_UTILS_HPP_INCLUDED_
#define SENSORS_WIC_UTILS_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/DUNE.hpp>

// ISO C++ 11 headers.
#include <string>

// Library headers.
#include <CameraSerialSettings.h>
#include <glib.h>
#include <gst/gst.h>

// Local headers.
#include "Constants.hpp"

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    typedef CameraSerialSettings CSS;

    inline std::string
    toStr(const CSS::CameraSpeed fps)
    {
      switch (fps)
      {
        case CSS::CameraSpeed::_9Hz:
          return WIC_FPS_9_HZ;
        case CSS::CameraSpeed::_30Hz:
          return WIC_FPS_30_HZ;
        case CSS::CameraSpeed::_60Hz:
          return WIC_FPS_60_HZ;
        default:
          return "unknown";
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
          return WIC_DOM_NONE;
        case CSS::DigitalOutputModes::XPMode:
          return WIC_DOM_XP;
        case CSS::DigitalOutputModes::LVDSMode:
          return WIC_DOM_LVDS;
        case CSS::DigitalOutputModes::CMOSBitDepth:
          return WIC_DOM_CMOS_BIT_DEPTH;
        case CSS::DigitalOutputModes::LVDSBitDepth:
          return WIC_DOM_LVDS_BIT_DEPTH;
        default:
          return "unknown";
      }
    }

    inline std::string
    toStr(const CSS::RangeModes range)
    {
      switch (range)
      {
        case CSS::RangeModes::Low:
          return WIC_RANGE_LOW;
        case CSS::RangeModes::Middle:
          return WIC_RANGE_MIDDLE;
        case CSS::RangeModes::High:
          return WIC_RANGE_HIGH;
        default:
          return "unknown";
      }
    }

    inline std::string
    toStr(const CSS::FFCModes ffc)
    {
      switch (ffc)
      {
        case CSS::FFCModes::Manual:
          return WIC_FFC_MANUAL;
        case CSS::FFCModes::Auto:
          return WIC_FFC_AUTO;
        case CSS::FFCModes::External:
          return WIC_FFC_EXTERNAL;
        default:
          return "unknown";
      }
    }

    inline std::string
    toStr(const CSS::XPBusModes xp)
    {
      switch (xp)
      {
        case CSS::XPBusModes::Disabled:
          return WIC_XPBUS_DISABLED;
        case CSS::XPBusModes::BT656:
          return WIC_XPBUS_BT656;
        case CSS::XPBusModes::CMOS:
          return WIC_XPBUS_CMOS;
        default:
          return "unknown";
      }
    }

    inline std::string
    toStr(const CSS::DigitalOutputDepth dod)
    {
      switch (dod)
      {
        case CSS::DigitalOutputDepth::Bits8b:
          return WIC_DOD_8_BIT_GRAYSCALE;
        case CSS::DigitalOutputDepth::Bit8bBayer:
          return WIC_DOD_8_BIT_BAYER;
        case CSS::DigitalOutputDepth::Bits14b:
          return WIC_DOD_14_BIT_RAW;
        case CSS::DigitalOutputDepth::Bit16bYCbCr:
          return WIC_DOD_16_BIT_YCBCR;
        default:
          return "unknown";
      }
    }

    inline std::string
    toStr(const CSS::Palettes palette)
    {
      switch (palette)
      {
        case CSS::Palettes::WhiteHot:
          return WIC_PALETTE_WHITE_HOT;
        case CSS::Palettes::BlackHot:
          return WIC_PALETTE_BLACK_HOT;
        case CSS::Palettes::Fusion:
          return WIC_PALETTE_FUSION;
        case CSS::Palettes::RainBow:
          return WIC_PALETTE_RAINBOW;
        case CSS::Palettes::Globow:
          return WIC_PALETTE_GLOBOW;
        case CSS::Palettes::Ironbow1:
          return WIC_PALETTE_IRONBOW1;
        case CSS::Palettes::Ironbow2:
          return WIC_PALETTE_IRONBOW2;
        case CSS::Palettes::Sepia:
          return WIC_PALETTE_SEPIA;
        case CSS::Palettes::Color1:
          return WIC_PALETTE_COLOR1;
        case CSS::Palettes::Color2:
          return WIC_PALETTE_COLOR2;
        case CSS::Palettes::Icefire:
          return WIC_PALETTE_ICEFIRE;
        case CSS::Palettes::Rain:
          return WIC_PALETTE_RAIN;
        case CSS::Palettes::RedHot:
          return WIC_PALETTE_REDHOT;
        case CSS::Palettes::GreenHot:
          return WIC_PALETTE_GREENHOT;
        default:
          return "unknown";
      }
    }

    inline std::string
    toStr(const CSS::AGCTypes agc)
    {
      switch (agc)
      {
        case CSS::AGCTypes::PlateauHistogram:
          return WIC_AGC_PLATEAU_HISTOGRAM;
        case CSS::AGCTypes::OnceBright:
          return WIC_AGC_ONCE_BRIGHT;
        case CSS::AGCTypes::AutoBright:
          return WIC_AGC_AUTO_BRIGHT;
        case CSS::AGCTypes::Manual:
          return WIC_AGC_MANUAL;
        case CSS::AGCTypes::NotDefined:
          return WIC_AGC_NOT_DEFINED;
        case CSS::AGCTypes::LinearAGC:
          return WIC_AGC_LINEAR_AGC;
        default:
          return "unknown";
      }
    }

    inline std::string
    toStr(const CSS::VideoColorModes vcm)
    {
      switch (vcm)
      {
        case CSS::VideoColorModes::Monochrome:
          return WIC_VCM_MONOCHROME;
        case CSS::VideoColorModes::Color:
          return WIC_VCM_COLOR;
        default:
          return "unknown";
      }
    }

    inline std::string
    toStr(const bool value)
    {
      return value ? "true" : "false";
    }

    inline std::string
    toStr(const uint16_t value)
    {
      return String::str("%d", value);
    }

    inline CSS::RangeModes
    toRange(const std::string& range_str)
    {
      if (range_str == WIC_RANGE_LOW)
        return CSS::RangeModes::Low;
      else if (range_str == WIC_RANGE_MIDDLE)
        return CSS::RangeModes::Middle;
      else if (range_str == WIC_RANGE_HIGH)
        return CSS::RangeModes::High;
      else
        return CSS::RangeModes::Low;
    }

    inline CSS::FFCModes
    toFFC(const std::string& ffc_str)
    {
      if (ffc_str == WIC_FFC_MANUAL)
        return CSS::FFCModes::Manual;
      else if (ffc_str == WIC_FFC_AUTO)
        return CSS::FFCModes::Auto;
      else if (ffc_str == WIC_FFC_EXTERNAL)
        return CSS::FFCModes::External;
      else
        return CSS::FFCModes::Manual;
    }

    inline CSS::DigitalOutputDepth
    toDOD(const std::string& dod_str)
    {
      if (dod_str == WIC_DOD_8_BIT_GRAYSCALE)
        return CSS::DigitalOutputDepth::Bits8b;
      else if (dod_str == WIC_DOD_8_BIT_BAYER)
        return CSS::DigitalOutputDepth::Bit8bBayer;
      else if (dod_str == WIC_DOD_14_BIT_RAW)
        return CSS::DigitalOutputDepth::Bits14b;
      else if (dod_str == WIC_DOD_16_BIT_YCBCR)
        return CSS::DigitalOutputDepth::Bit16bYCbCr;
      else
        return CSS::DigitalOutputDepth::Bits8b;
    }

    inline CSS::Palettes
    toPalette(const std::string& palette_str)
    {
      if (palette_str == WIC_PALETTE_WHITE_HOT)
        return CSS::Palettes::WhiteHot;
      else if (palette_str == WIC_PALETTE_BLACK_HOT)
        return CSS::Palettes::BlackHot;
      else if (palette_str == WIC_PALETTE_FUSION)
        return CSS::Palettes::Fusion;
      else if (palette_str == WIC_PALETTE_RAINBOW)
        return CSS::Palettes::RainBow;
      else if (palette_str == WIC_PALETTE_GLOBOW)
        return CSS::Palettes::Globow;
      else if (palette_str == WIC_PALETTE_IRONBOW1)
        return CSS::Palettes::Ironbow1;
      else if (palette_str == WIC_PALETTE_IRONBOW2)
        return CSS::Palettes::Ironbow2;
      else if (palette_str == WIC_PALETTE_SEPIA)
        return CSS::Palettes::Sepia;
      else if (palette_str == WIC_PALETTE_COLOR1)
        return CSS::Palettes::Color1;
      else if (palette_str == WIC_PALETTE_COLOR2)
        return CSS::Palettes::Color2;
      else if (palette_str == WIC_PALETTE_ICEFIRE)
        return CSS::Palettes::Icefire;
      else if (palette_str == WIC_PALETTE_RAIN)
        return CSS::Palettes::Rain;
      else if (palette_str == WIC_PALETTE_REDHOT)
        return CSS::Palettes::RedHot;
      else if (palette_str == WIC_PALETTE_GREENHOT)
        return CSS::Palettes::GreenHot;
      else
        return CSS::Palettes::WhiteHot;
    }

    inline CSS::AGCTypes
    toAGC(const std::string& agc_str)
    {
      if (agc_str == WIC_AGC_PLATEAU_HISTOGRAM)
        return CSS::AGCTypes::PlateauHistogram;
      else if (agc_str == WIC_AGC_ONCE_BRIGHT)
        return CSS::AGCTypes::OnceBright;
      else if (agc_str == WIC_AGC_AUTO_BRIGHT)
        return CSS::AGCTypes::AutoBright;
      else if (agc_str == WIC_AGC_MANUAL)
        return CSS::AGCTypes::Manual;
      else if (agc_str == WIC_AGC_NOT_DEFINED)
        return CSS::AGCTypes::NotDefined;
      else if (agc_str == WIC_AGC_LINEAR_AGC)
        return CSS::AGCTypes::LinearAGC;
      else
        return CSS::AGCTypes::NotDefined;
    }

    inline CSS::VideoColorModes
    toVCM(const std::string& vcm_str)
    {
      if (vcm_str == WIC_VCM_MONOCHROME)
        return CSS::VideoColorModes::Monochrome;
      else if (vcm_str == WIC_VCM_COLOR)
        return CSS::VideoColorModes::Color;
      else
        return CSS::VideoColorModes::Monochrome;
    }
  }
}

#endif
