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

#ifndef SENSORS_WIC_IMAGE_PROCESSOR_HPP_INCLUDED_
#define SENSORS_WIC_IMAGE_PROCESSOR_HPP_INCLUDED_

// C++ STL headers.
#include <cmath>
#include <vector>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Library headers.
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>

// Local headers.
#include "Constants.hpp"
#include "PaletteLoader.hpp"

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    //! Enum representing the method to use for contrast correction.
    enum class ContrastCorrectionMethod
    {
      //! No contrast correction.
      NONE,
      //! Gamma correction.
      GAMMA_CORRECTION,
      //! CLAHE (Contrast-Limited Adaptive Histogran Equalization).
      CLAHE,
    };

    //! Enum representing the color scaling method to use.
    enum class ColorScalingMethod
    {
      //! Automatic color scaling using the image min/max.
      AUTO_MIN_MAX,
      //! Automatic color scaling using the mean and standard deviation for normalization.
      AUTO_STD_DEV,
      //! Manual color scaling using limits from the user.
      MANUAL,
    };

    //! Struct holding the contrast correction settings.
    struct ContrastCorrectionSettings
    {
      //! Contrast correction method.
      ContrastCorrectionMethod method;
      //! Exponent to use in the gamma correction.
      double gamma_correction_factor;
      //! CLAHE tile size.
      int clahe_tile_size;
      //! CLAHE clip limit.
      double clahe_clip_limit;
    };

    //! Struct holding the color settings.
    struct ColorSettings
    {
      //! The color palette to use.
      Palette palette;
      //! The color scaling mode to use.
      ColorScalingMethod color_scaling_method;
      //! Amount of standard deviations to use in automatic scaling mode with standard deviation.
      double auto_color_scaling_std_dev_factor;
      //! Manual colors caling mode min temperature.
      double manual_color_scaling_min_temperature;
      //! Manual color scaling mode max temperature.
      double manual_color_scaling_max_temperature;
      //! Whether or not to invert the colors.
      bool invert_colors {false};
      //! Slope for the RAW->T conversion.
      double raw_to_temp_slope;
      //! Offset for the RAW->T conversion.
      double raw_to_temp_offset;
    };

    //! Struct holding the parameters for drawing the color scale bar on the image.
    struct ColorScaleParameters
    {
      //! The positions of the border corner points, in pixels.
      int border_x_min;
      int border_x_max;
      int border_y_min;
      int border_y_max;
      //! The width of the notches, in pixels.
      int notch_width;
      //! The horizontal spacing between the values and the notches, in pixels.
      int values_notch_spacing;
    };

    //! Struct holding the current frame properties.
    struct CurrentFrameProperties
    {
      //! The min & max temperatures in the current frame.
      double temp_min;
      double temp_max;
    };

    inline ContrastCorrectionMethod
    contrastFromStr(const std::string& method)
    {
      if (method == c_contrast_correction_gamma)
        return ContrastCorrectionMethod::GAMMA_CORRECTION;
      else if (method == c_contrast_correction_clahe)
        return ContrastCorrectionMethod::CLAHE;
      else
        return ContrastCorrectionMethod::NONE;
    }

    inline std::string
    toStr(const ContrastCorrectionMethod method)
    {
      if (method == ContrastCorrectionMethod::GAMMA_CORRECTION)
        return c_contrast_correction_gamma;
      else if (method == ContrastCorrectionMethod::CLAHE)
        return c_contrast_correction_clahe;
      else
        return c_contrast_correction_none;
    }

    inline ColorScalingMethod
    colorScalingFromStr(const std::string& method)
    {
      if (method == c_color_scaling_method_full_scale)
        return ColorScalingMethod::AUTO_MIN_MAX;
      else if (method == c_color_scaling_method_smoothed)
        return ColorScalingMethod::AUTO_STD_DEV;
      else
        return ColorScalingMethod::MANUAL;
    }

    inline std::string
    toStr(const ColorScalingMethod method)
    {
      if (method == ColorScalingMethod::AUTO_MIN_MAX)
        return c_color_scaling_method_full_scale;
      else if (method == ColorScalingMethod::AUTO_STD_DEV)
        return c_color_scaling_method_smoothed;
      else
        return c_color_scaling_method_manual;
    }

    class ImageProcessor
    {
    public:
      ImageProcessor(Task* parent):
        m_clahe {cv::createCLAHE()},
        m_parent {parent}
      {
      }

      void
      init(const unsigned width_in_pixels, const unsigned height_in_pixels)
      {
        m_width_in_pixels = width_in_pixels;
        m_height_in_pixels = height_in_pixels;
        m_input_image = cv::Mat(m_height_in_pixels, m_width_in_pixels, CV_16U);
        m_processed_image = cv::Mat(height_in_pixels, width_in_pixels, CV_8UC3);

        double border_distance_from_right_edge
          = static_cast<double>(m_width_in_pixels) * c_colorscale_border_fractional_distance_from_right_edge;
        double border_height
          = static_cast<double>(m_height_in_pixels) * c_colorscale_border_fractional_height;

        m_color_scale_params.border_x_max = m_width_in_pixels - static_cast<int>(border_distance_from_right_edge);
        m_color_scale_params.border_x_min = m_color_scale_params.border_x_max - c_colorscale_width_in_pixels;
        m_color_scale_params.border_y_min = ((m_height_in_pixels - static_cast<int>(border_height)) / 2);
        m_color_scale_params.border_y_max = m_color_scale_params.border_y_min + static_cast<int>(border_height);
      }

      void
      setPictureDirectory(const FileSystem::Path& picture_directory)
      {
        m_picture_directory = picture_directory;
      }

      void
      updateColorSettings(const ColorSettings& settings)
      {
        if (settings.color_scaling_method != m_color_settings.color_scaling_method)
          m_parent->inf("set color scaling method: %s", toStr(settings.color_scaling_method).c_str());

        // This operation is expensive, so only perform if necessary
        if (settings.palette != m_color_settings.palette ||
            settings.invert_colors != m_color_settings.invert_colors)
          updateColorPalette(settings.palette, settings.invert_colors);

        m_color_settings = settings;
      }

      void
      updateContrastCorrectionSettings(const ContrastCorrectionSettings& settings)
      {
        if (settings.method != m_contrast_correction_settings.method)
          m_parent->inf("set contrast correction method: %s", toStr(settings.method).c_str());

        // This operation is expensive, so only perform if necessary
        if (settings.gamma_correction_factor != m_contrast_correction_settings.gamma_correction_factor)
          createGammaCorrectionLUT(settings.gamma_correction_factor);

        m_clahe->setClipLimit(settings.clahe_clip_limit);
        m_clahe->setTilesGridSize(cv::Size(settings.clahe_tile_size, settings.clahe_tile_size));
        m_contrast_correction_settings = settings;
      }

      void
      processRawImage(uint8_t* raw_buffer, const bool draw_overlay)
      {
        m_input_image.data = raw_buffer;
        applyColorPalette();
        applyContrastCorrection();

        //! TODO:
        //! Currently, the whole color scale is re-drawn every time.
        //! We could improve that so that only the values (text) are re-drawn every frame, all the rest only on change
        //! or when necessary.
        if (draw_overlay)
        {
          drawColorScale();
          drawColorScaleBorders();
          drawColorScaleValues();
        }
      }

      uint8_t*
      processedImage() const
      {
        return m_processed_image.data;
      }

      void
      saveProcessedImage()
      {
        std::string filename = "wic-picture-"
                               + Time::Format::getDateSafe()
                               + "-"
                               + Time::Format::getTimeSafe()
                               + ".jpg";
        std::string filepath = (m_picture_directory / filename).str();

        cv::Mat bgr_image;
        cv::cvtColor(m_processed_image, bgr_image, cv::COLOR_RGB2BGR);

        try
        {
          cv::imwrite(filepath, bgr_image);
          m_parent->inf("saved image to %s", filepath.c_str());
        }
        catch (const cv::Exception& e)
        {
          m_parent->err("failed to save image to %s: %s", filepath.c_str(), e.what());
        }
      }

    private:
      void
      createGammaCorrectionLUT(const double correction_factor)
      {
        m_gamma_lut = cv::Mat(1, c_palette_size, CV_8UC1);
        unsigned char* lut_data = m_gamma_lut.data;
        for (int i = 0; i < c_palette_size; i++)
          lut_data[i] =
            static_cast<uint8_t>(std::pow(static_cast<double>(i) / 255.0, correction_factor) * 255.0);
      }

      void
      updateColorPalette(const Palette& palette, const bool invert_colors)
      {
        m_palette = cv::Mat(1, palette.size(), CV_8UC3);
        m_palette_manual = cv::Mat(1, palette.size(), CV_8UC3);
        const int palette_size = (int)palette.size();
        for (int i = 0; i < palette_size; i++)
        {
          RGBColor color = invert_colors ? palette.at(palette_size - 1 - i) : palette.at(i);
          m_palette.at<cv::Vec3b>(0, i) = cv::Vec3b(color[0], color[1], color[2]);
          m_palette_manual.at<cv::Vec3b>(0, i) = cv::Vec3b(color[0], color[1], color[2]);
        }

        m_palette_manual.at<cv::Vec3b>(0, palette_size - 1) = m_palette_manual.at<cv::Vec3b>(0, 0);
      }

      void
      applyColorPalette()
      {
        double image_max;
        double image_min;
        getColorScalingMinMax(image_min, image_max);

        const double slope = 255.0 / (image_max - image_min);
        const double intercept = - slope * image_min;

        cv::Mat scaled_image = cv::Mat(m_width_in_pixels, m_height_in_pixels, CV_8UC1);
        m_input_image.convertTo(scaled_image, CV_8UC3, slope, intercept);
        cv::Mat scaled_image_3_channels = cv::Mat(m_width_in_pixels, m_height_in_pixels, CV_8UC3);
        cv::cvtColor(scaled_image, scaled_image_3_channels, cv::COLOR_GRAY2RGB);
        if (m_color_settings.color_scaling_method == ColorScalingMethod::MANUAL)
          cv::LUT(scaled_image_3_channels, m_palette_manual, m_processed_image);
        else
          cv::LUT(scaled_image_3_channels, m_palette, m_processed_image);
      }

      void
      getColorScalingMinMax(double& min, double& max)
      {
        cv::Mat input_image_temperature = cv::Mat(m_width_in_pixels, m_height_in_pixels, CV_32F);
        m_input_image.convertTo(input_image_temperature, CV_32F,
                                m_color_settings.raw_to_temp_slope, m_color_settings.raw_to_temp_offset);

        if (m_color_settings.color_scaling_method == ColorScalingMethod::AUTO_MIN_MAX)
        {
          cv::minMaxIdx(input_image_temperature, &min, &max);
        }
        else if (m_color_settings.color_scaling_method == ColorScalingMethod::AUTO_STD_DEV)
        {
          cv::Scalar image_mean;
          cv::Scalar image_stddev;
          cv::meanStdDev(input_image_temperature, image_mean, image_stddev);
          min = image_mean.val[0] - m_color_settings.auto_color_scaling_std_dev_factor * image_stddev.val[0];
          max = image_mean.val[0] + m_color_settings.auto_color_scaling_std_dev_factor * image_stddev.val[0];
        }
        else
        {
          min = m_color_settings.manual_color_scaling_min_temperature;
          max = m_color_settings.manual_color_scaling_max_temperature;
        }

        m_current_frame_properties.temp_min = min;
        m_current_frame_properties.temp_max = max;

        min = (min - m_color_settings.raw_to_temp_offset) * (1.0 / m_color_settings.raw_to_temp_slope);
        max = (max - m_color_settings.raw_to_temp_offset) * (1.0 / m_color_settings.raw_to_temp_slope);
      }

      void
      applyContrastCorrection()
      {
        if (m_contrast_correction_settings.method == ContrastCorrectionMethod::NONE)
          return;

        cv::Mat lab_unprocessed;
        cv::Mat lab_components[3];
        cv::Mat lab_L_processed;
        cv::Mat lab_processed;

        cv::cvtColor(m_processed_image, lab_unprocessed, cv::COLOR_RGB2Lab);
        cv::split(lab_unprocessed, lab_components);
        if (m_contrast_correction_settings.method == ContrastCorrectionMethod::GAMMA_CORRECTION)
          cv::LUT(lab_components[0], m_gamma_lut, lab_L_processed);
        if (m_contrast_correction_settings.method == ContrastCorrectionMethod::CLAHE)
          m_clahe->apply(lab_components[0], lab_L_processed);
        std::vector<cv::Mat> lab_vector = {lab_L_processed, lab_components[1], lab_components[2]};
        cv::merge(lab_vector, lab_processed);
        cv::cvtColor(lab_processed, m_processed_image, cv::COLOR_Lab2RGB);
      }

      void
      drawColorScale()
      {
        const int color_scale_height {m_color_scale_params.border_y_max - m_color_scale_params.border_y_min};
        for (int i = 0; i < color_scale_height; i++)
        {
          const int index {i * c_palette_size / color_scale_height};
          cv::Scalar color = m_palette.at<cv::Vec3b>(0, c_palette_size - index);
          cv::Point start_point {m_color_scale_params.border_x_min, m_color_scale_params.border_y_min + i};
          cv::Point end_point {m_color_scale_params.border_x_max, m_color_scale_params.border_y_min + i};
          cv::line(m_processed_image, start_point, end_point, color, 1, cv::LINE_AA);
        }
      }

      void
      drawColorScaleBorders()
      {
        //! Draw the border twice in different colors so that it's always visible
        drawColorScaleBorder(cv::Scalar(0.0, 0.0, 0.0), 2);
        drawColorScaleBorder(cv::Scalar(255.0, 255.0, 255.0), 1);
      }

      void
      drawColorScaleValues()
      {
        //! Draw the values twice in different colors so that they are always visible
        drawColorScaleValues(cv::Scalar(0.0, 0.0, 0.0), 2);
        drawColorScaleValues(cv::Scalar(255.0, 255.0, 255.0), 1);
      }

      void
      drawColorScaleBorder(const cv::Scalar& color, const int thickness)
      {
        const int color_scale_height {m_color_scale_params.border_y_max - m_color_scale_params.border_y_min};
        const cv::Point border_corner_1 {m_color_scale_params.border_x_min, m_color_scale_params.border_y_min};
        const cv::Point border_corner_2 {m_color_scale_params.border_x_max, m_color_scale_params.border_y_max};

        cv::rectangle(m_processed_image, border_corner_1, border_corner_2, color, thickness, cv::LINE_AA);
        for (int i = 0; i < c_colorscale_amount_of_notches; i++)
        {
          const int notch_y = static_cast<int>(std::round(
                                static_cast<float>(m_color_scale_params.border_y_min)
                                + i * (static_cast<float>(color_scale_height) / (c_colorscale_amount_of_notches - 1))
                              ));
          const int notch_x_max = m_color_scale_params.border_x_max + c_colorscale_notch_width_in_pixels;
          const cv::Point notch_p1 {m_color_scale_params.border_x_max, notch_y};
          const cv::Point notch_p2 {notch_x_max, notch_y};
          cv::line(m_processed_image, notch_p1, notch_p2, color, thickness, cv::LINE_AA);
        }
      }

      void
      drawColorScaleValues(const cv::Scalar& color, const int thickness)
      {
        const int color_scale_height {m_color_scale_params.border_y_max - m_color_scale_params.border_y_min};
        int baseline;
        cv::Size text_size = cv::getTextSize(String::str("%.1f", m_current_frame_properties.temp_max),
                                             cv::FONT_HERSHEY_SIMPLEX,
                                             0.3,
                                             thickness,
                                             &baseline);

        for (int i = 0; i < c_colorscale_amount_of_notches; i++)
        {
          const int notch_y = m_color_scale_params.border_y_min
                              + i * (color_scale_height / (c_colorscale_amount_of_notches - 1));
          const int notch_x_max = m_color_scale_params.border_x_max + c_colorscale_notch_width_in_pixels;
          const double temp = m_current_frame_properties.temp_max
                              - i * (m_current_frame_properties.temp_max - m_current_frame_properties.temp_min) /
                              (c_colorscale_amount_of_notches - 1);
          const int text_offset_y = text_size.height / 2;

          std::string symbol {""};
          std::string text {String::str("%.1f", temp)};
          if (m_color_settings.color_scaling_method == ColorScalingMethod::AUTO_STD_DEV)
          {
            if (i == 0)
              symbol = ">";
            else if (i == c_colorscale_amount_of_notches - 1)
              symbol = "<";
          }

          cv::putText(m_processed_image,
                      String::str("%s%.1f", symbol.c_str(), temp),
                      cv::Point(notch_x_max + c_colorscale_values_horizontal_offset_in_pixels, notch_y + text_offset_y),
                      cv::FONT_HERSHEY_SIMPLEX,
                      0.3, color, thickness, cv::LINE_AA);
        }
      }

      //! Width of a frame, in pixels.
      int m_width_in_pixels;
      //! Height of a frame, in pixels.
      int m_height_in_pixels;
      //! The current seetings to use for contrast correction.
      ContrastCorrectionSettings m_contrast_correction_settings;
      //! The current settings to use for coloring the image.
      ColorSettings m_color_settings;
      //! The parameters for drawing the color scale bar.
      ColorScaleParameters m_color_scale_params;
      //! The current frame properties.
      CurrentFrameProperties m_current_frame_properties;
      //! The current color palette lookup-table.
      cv::Mat m_palette;
      //! The current color palette lookup-table for the special case of manual color scaling.
      cv::Mat m_palette_manual;
      //! The current gamma contrast correction lookup-table.
      cv::Mat m_gamma_lut;
      //! The (optional) overlay for the image.
      cv::Mat m_overlay;
      //! The current input image.
      cv::Mat m_input_image;
      //! The latest processed image.
      cv::Mat m_processed_image;
      //! Pointer to a CLAHE object for performing histogram equilization.
      cv::Ptr<cv::CLAHE> m_clahe;
      //! The directory where to store pictures.
      FileSystem::Path m_picture_directory;
      //! Pointer to the parent Task.
      Task* m_parent;
    };
  }
}

#endif
