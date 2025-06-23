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

// C++ STL headers.
#include <string>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    //! Task restart delay (seconds).
    constexpr float c_restart_delay {5.0};
    //! Timeout after which pipeline start is considered failed (seconds).
    constexpr float c_pipeline_start_timeout {10.0};
    //! Stack size for this thread (default is too small for WIC SDK 1.1.0).
    constexpr size_t c_thread_stack_size {8192U * 1024U};

    //! Constants for the values of list parameters.
    const std::string c_contrast_correction_none {"NO CONTRAST CORRECTION"};
    const std::string c_contrast_correction_gamma {"GAMMA CORRECTION"};
    const std::string c_contrast_correction_clahe {"HISTOGRAM EQUILIZATION (CLAHE)"};
    const std::string c_color_scaling_method_full_scale {"AUTOMATIC - FULL SCALE (MIN-MAX)"};
    const std::string c_color_scaling_method_smoothed {"AUTOMATIC - SMOOTHED"};
    const std::string c_color_scaling_method_manual {"MANUAL"};

    //! Pipeline constants
    constexpr int c_pipeline_queue_stream_max_size_buffers {1};
    constexpr int c_pipeline_queue_record_max_size_buffers {1};
    constexpr int c_pipeline_x264enc_tune {0x04};
    constexpr int c_pipeline_x264enc_speed_preset {0x01};
    constexpr int c_pipeline_x264enc_bitrate {4000};
    constexpr int c_pipeline_x264enc_stream_key_int_max {0};
    constexpr int c_pipeline_x264enc_record_key_int_max {10};
    constexpr int c_pipeline_nvv4l2h264enc_bitrate {c_pipeline_x264enc_bitrate * 1000};
    constexpr int c_pipeline_nvv4l2h264enc_insert_sps_pps {1};
    constexpr bool c_pipeline_nvv4l2h264enc_maxperf_enable {true};
    constexpr int c_pipeline_stream_payloader_config_interval {1};

    //! Path (relative to DUNE etc/ folder) to the directory containing the color palette files.
    const FileSystem::Path c_color_palette_dir {"evologics/sonobot-v5/payload/wic/color-palettes"};
    //! File extension of Workswell palette files.
    const std::string c_palette_file_ext {"plt"};
    //! Name of the default palette.
    const std::string c_default_palette_name {"None"};
    //! Expected amount of RGB-triplets in a Workswell palette file.
    constexpr int c_palette_size {256U};
    //! Constants for the image overlay.
    constexpr double c_colorscale_border_fractional_distance_from_right_edge {0.08};
    constexpr double c_colorscale_border_fractional_height {0.8};
    constexpr int c_colorscale_width_in_pixels {7};
    constexpr int c_colorscale_amount_of_notches {5};
    constexpr int c_colorscale_notch_width_in_pixels {4};
    constexpr int c_colorscale_values_horizontal_offset_in_pixels {5};
  }
}

#endif
