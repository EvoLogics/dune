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

#ifndef SENSORS_WIC_PALETTE_LOADER_HPP_INCLUDED_
#define SENSORS_WIC_PALETTE_LOADER_HPP_INCLUDED_

// ISO C++ 11 headers.
#include <array>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include "Constants.hpp"

namespace Sensors
{
  namespace WIC
  {
    using DUNE_NAMESPACES;

    //! Convenience typedefs.
    typedef std::array<int, 3> RGBColor;
    typedef std::vector<RGBColor> Palette;

    class PaletteLoader
    {
    public:
      PaletteLoader(const FileSystem::Path& palette_dir, Task* parent):
        m_parent {parent}
      {
        createDefaultPalette();
        selectDefaultPalette();
        findPalettes(palette_dir);
      }

      Palette&
      palette()
      {
        return m_selected_palette;
      }

      std::vector<std::string>
      paletteNames() const
      {
        std::vector<std::string> palette_names;
        palette_names.push_back(c_default_palette_name);
        for (const auto& palette_path : m_palette_paths)
          palette_names.push_back(palette_path.first);
        return palette_names;
      }

      void
      loadPalette(const std::string& palette_name)
      {
        if (palette_name == m_current_palette_name)
        {
          m_parent->debug("palette '%s' already loaded, not loading again", palette_name.c_str());
          return;
        }

        if (palette_name == c_default_palette_name)
        {
          selectDefaultPalette();
          return;
        }
        ReturnValue rv = loadPaletteFromFile(palette_name);
        if (!rv.ok)
          m_parent->err("%s", rv.msg.c_str());
      }

    private:
      struct ReturnValue
      {
        bool ok;
        std::string msg;
      };

      void
      createDefaultPalette()
      {
        for (int i = 0; i < c_palette_size; i++)
          m_default_palette.push_back({i, i, i});
      }

      void
      selectDefaultPalette()
      {
        m_selected_palette = m_default_palette;
        m_parent->inf("loaded default palette");
        m_current_palette_name = "default";
      }

      void
      findPalettes(const FileSystem::Path& palette_dir)
      {
        if (!palette_dir.isDirectory())
          return;

        std::vector<FileSystem::Path> files;
        palette_dir.contents(files);

        if (files.empty())
          return;

        for (const FileSystem::Path& file : files)
        {
          if (file.extension() != c_palette_file_ext)
            continue;

          std::string file_basename = file.basename().str();
          std::string palette_name = file_basename.substr(0, file_basename.size() - c_palette_file_ext.size() - 1);
          m_palette_paths[palette_name] = file;
          m_parent->debug("found palette '%s'", palette_name.c_str());
        }
      }

      ReturnValue
      loadPaletteFromFile(const std::string& palette_name)
      {
        std::string error_prefix = "cannot load palette '" + palette_name + "'";

        FileSystem::Path palette_path;
        try
        {
          palette_path = m_palette_paths.at(palette_name);
        }
        catch(const std::out_of_range& e)
        {
          return {false, String::str("%s: unknown palette", error_prefix.c_str())};
        }

        if (!palette_path.isFile())
          return {false, String::str("%s: file does not exist: %s", error_prefix.c_str(), palette_path.str().c_str())};

        m_parent->debug("loading palette from file: %s", palette_path.str().c_str());

        std::ifstream filestream {palette_path.str()};
        std::string line;
        Palette palette;

        int line_count {0};
        while (std::getline(filestream, line) && line_count < c_palette_size)
        {
          RGBColor color;
          ReturnValue rv = getRGBFromLine(line, color);

          if (!rv.ok)
            return {false, String::str("%s: %s", error_prefix.c_str(), rv.msg.c_str())};

          palette.push_back(color);
          line_count++;
        }

        if (palette.size() != c_palette_size)
          return {false, String::str("%s: expected %d RGB-triplets in palette (actual: %d)",
                                     error_prefix.c_str(),
                                     c_palette_size,
                                     palette.size())};

        m_selected_palette = palette;
        m_current_palette_name = palette_name;
        m_parent->inf("loaded palette '%s'", palette_name.c_str());

        return {true, ""};
      }

      ReturnValue
      getRGBFromLine(std::string line, RGBColor& color)
      {
        // the lines in Workswell palette files have inconsistent formatting
        // therefore, we bring each line to a known state before proceeding
        line = String::replace(line, '\t', "");
        line = String::replace(line, '\r', "");
        line = String::replace(line, '\n', "");
        line = String::replace(line, ' ', "");
        if (String::endsWith(line, ";"))
          line.pop_back();

        std::vector<std::string> parts;
        String::split(line, ";", parts);

        if (parts.size() != 3U)
          return {false, "cannot parse RGB: at least one line in the file does not describe a RGB-triplet"};

        try
        {
          color[0] = std::stoi(parts[0]);
          color[1] = std::stoi(parts[1]);
          color[2] = std::stoi(parts[2]);
        }
        catch (const std::exception& e)
        {
          return {false, "cannot parse RGB: exception while converting string to integer: " + std::string(e.what())};
        }

        return {true, ""};
      }

      std::string m_current_palette_name;
      Palette m_selected_palette;
      Palette m_default_palette;
      std::map<std::string, FileSystem::Path> m_palette_paths;
      Task* m_parent;
    };

  }
}

#endif
