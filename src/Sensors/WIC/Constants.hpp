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

#ifndef SENSORS_WIC_CONSTANTS_HPP_INCLUDED_
#define SENSORS_WIC_CONSTANTS_HPP_INCLUDED_

// ISO C++ 11 headers.
#include <cstdint>
#include <string>

namespace Sensors
{
  namespace WIC
  {

    //! Restart delay (seconds).
    constexpr float c_restart_delay {5.0};
    //! Timeout for (re)starting the pipeline
    constexpr float c_pipeline_start_timeout {10.0};
    //! Stack size for this thread (default is too small for WIC SDK)
    constexpr size_t c_thread_stack_size {8192U * 1024U};

    //! Pipeline constants
    constexpr int c_pipeline_videorate_max_rate                 = 10;
    constexpr int c_pipeline_x264enc_tune                       = 0x04;
    constexpr int c_pipeline_x264enc_speed_preset               = 0x01;
    constexpr int c_pipeline_x264enc_qp_min                     = 20;
    constexpr int c_pipeline_x264enc_key_int_max                = 1;
    constexpr int c_pipeline_nvv4l2h264enc_bitrate              = 157286400; // TODO: changeme to reasonable value
    constexpr int c_pipeline_nvv4l2h264enc_insert_sps_pps       = 1;
    constexpr bool c_pipeline_nvv4l2h264enc_maxperf_enable      = true;

    //! WIC Speed
    constexpr char WIC_FPS_9_HZ[]              = "9 Hz";
    constexpr char WIC_FPS_30_HZ[]             = "30 Hz";
    constexpr char WIC_FPS_60_HZ[]             = "60 Hz";

    //! WIC Digital Output Mode
    constexpr char WIC_DOM_NONE[]              = "None";
    constexpr char WIC_DOM_XP[]                = "XP Mode";
    constexpr char WIC_DOM_LVDS[]              = "LVDS Mode";
    constexpr char WIC_DOM_CMOS_BIT_DEPTH[]    = "CMOS Bit Depth";
    constexpr char WIC_DOM_LVDS_BIT_DEPTH[]    = "LVDS Bit Depth";

    //! WIC Range Mode
    constexpr char WIC_RANGE_LOW[]             = "Low";
    constexpr char WIC_RANGE_MIDDLE[]          = "Middle";
    constexpr char WIC_RANGE_HIGH[]            = "High";

    //! WIC FFC Mode
    constexpr char WIC_FFC_MANUAL[]            = "Manual";
    constexpr char WIC_FFC_AUTO[]              = "Auto";
    constexpr char WIC_FFC_EXTERNAL[]          = "External";

    //! WIC XPBus Mode
    constexpr char WIC_XPBUS_DISABLED[]        = "Disabled";
    constexpr char WIC_XPBUS_BT656[]           = "BT656";
    constexpr char WIC_XPBUS_CMOS[]            = "CMOS";

    //! WIC Digital Output Depth
    constexpr char WIC_DOD_8_BIT_GRAYSCALE[]   = "8 bit Grayscale";
    constexpr char WIC_DOD_8_BIT_BAYER[]       = "8 bit Bayer";
    constexpr char WIC_DOD_14_BIT_RAW[]        = "14 bit RAW";
    constexpr char WIC_DOD_16_BIT_YCBCR[]      = "16 bit YCbCr";

    //! WIC Palette
    constexpr char WIC_PALETTE_WHITE_HOT[]     = "WhiteHot";
    constexpr char WIC_PALETTE_BLACK_HOT[]     = "BlackHot";
    constexpr char WIC_PALETTE_FUSION[]        = "Fusion";
    constexpr char WIC_PALETTE_RAINBOW[]       = "RainBow";
    constexpr char WIC_PALETTE_GLOBOW[]        = "Globow";
    constexpr char WIC_PALETTE_IRONBOW1[]      = "Ironbow1";
    constexpr char WIC_PALETTE_IRONBOW2[]      = "Ironbow2";
    constexpr char WIC_PALETTE_SEPIA[]         = "Sepia";
    constexpr char WIC_PALETTE_COLOR1[]        = "Color1";
    constexpr char WIC_PALETTE_COLOR2[]        = "Color2";
    constexpr char WIC_PALETTE_ICEFIRE[]       = "Icefire";
    constexpr char WIC_PALETTE_RAIN[]          = "Rain";
    constexpr char WIC_PALETTE_REDHOT[]        = "RedHot";
    constexpr char WIC_PALETTE_GREENHOT[]      = "GreenHot";

    //! WIC AGC Type
    constexpr char WIC_AGC_PLATEAU_HISTOGRAM[] = "PlateauHistogram";
    constexpr char WIC_AGC_ONCE_BRIGHT[]       = "OnceBright";
    constexpr char WIC_AGC_AUTO_BRIGHT[]       = "AutoBright";
    constexpr char WIC_AGC_MANUAL[]            = "Manual";
    constexpr char WIC_AGC_NOT_DEFINED[]       = "NotDefined";
    constexpr char WIC_AGC_LINEAR_AGC[]        = "LinearAGC";
    
    //! WIC Video Color Mode (Monochrome, Color)
    constexpr char WIC_VCM_MONOCHROME[]        = "Monochrome";
    constexpr char WIC_VCM_COLOR[]             = "Color";

  }
}

#endif
