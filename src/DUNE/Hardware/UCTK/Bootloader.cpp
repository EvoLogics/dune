//***************************************************************************
// Copyright 2007-2013 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
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
// https://www.lsts.pt/dune/licence.                                        *
//***************************************************************************
// Author: Ricardo Martins                                                  *
//***************************************************************************

// ISO C++ 98 headers.
#include <cstdio>
#include <cstdarg>

// DUNE headers.
#include <DUNE/Algorithms/CRC8.hpp>
#include <DUNE/Hardware/UCTK/Bootloader.hpp>
#include <DUNE/Hardware/UCTK/FirmwareInfo.hpp>

namespace DUNE
{
  namespace Hardware
  {
    namespace UCTK
    {
      //! Size of fill chunk.
      static const unsigned c_fill_chunk_size = 32;

      Bootloader::Bootloader(Interface* itf, bool verbose):
        m_itf(itf),
        m_verbose(verbose)
      {
        title("Device");

        const FirmwareInfo& info(m_itf->getFirmwareInfo());
        print("%-20s: %s\n", "Firmware Name", info.name.c_str());
        print("%-20s: %u.%u.%u\n", "Firmware Version", info.major, info.minor,
              info.patch);

        getFlashInfo();
      }

      void
      Bootloader::program(const std::string& file_name)
      {
        title("Firmare");

        // Load iHEX file.
        IntelHEX ihex(file_name, m_page_size);

        // Compute program size.
        uint32_t size = ihex.getTable().size() * m_page_size;
        print("%-20s: %u\n", "Intel HEX - Size", size);

        // Compute program CRC.
        Algorithms::CRC8 crc(0x07);
        const IntelHEX::PageTable& table = ihex.getTable();
        IntelHEX::PageTable::const_iterator itr = table.begin();
        for (; itr != table.end(); ++itr)
          crc.putArray(&itr->second[0], itr->second.size());
        print("%-20s: 0x%02X\n", "Intel HEX - CRC8", crc.get());

        title("Programming");

        // Start upgrade procedure.
        m_frame.setId(PKT_ID_BOOT_UPGRADE_START);
        m_frame.setPayloadSize(5);
        m_frame.set(size, 0);
        m_frame.set(crc.get(), 4);
        if (!m_itf->sendFrame(m_frame))
          throw std::runtime_error("failed start upgrade procedure");

        // Program pages.
        IntelHEX::PageTable::const_iterator pitr = table.begin();
        for (; pitr != table.end(); ++pitr)
        {
          fillPage(pitr->first, pitr->second);
        }

        // End upgrade procedure.
        m_frame.setId(PKT_ID_BOOT_UPGRADE_END);
        m_frame.setPayloadSize(0);
        if (!m_itf->sendFrame(m_frame))
          throw std::runtime_error("failed to end upgrade procedure");

        reset();

        print("\nSuccess!\n\n");
      }

      void
      Bootloader::fillPage(unsigned page, const std::vector<uint8_t>& contents)
      {
        uint16_t offset = 0;
        unsigned chunk_count = contents.size() / c_fill_chunk_size;

        print("Page %u: ", page);

        m_frame.setId(PKT_ID_BOOT_FLASH_FILL);
        for (unsigned i = 0; i < chunk_count; ++i)
        {
          offset = i * c_fill_chunk_size;
          m_frame.set(offset, 0);
          std::memcpy(m_frame.getPayload() + 2, &contents[offset], c_fill_chunk_size);

          m_frame.setPayloadSize(c_fill_chunk_size + 2);
          if (!m_itf->sendFrame(m_frame))
            throw std::runtime_error("failed to fill page chunk");

          print(".");
        }

        // Write page.
        m_frame.setId(PKT_ID_BOOT_FLASH_WRITE);
        m_frame.setPayloadSize(4);
        m_frame.set<uint32_t>(page * m_page_size, 0);
        if (!m_itf->sendFrame(m_frame))
          throw std::runtime_error("failed to write flash page");

        print(" OK\n");
      }

      void
      Bootloader::print(const char* format, ...) const
      {
        if (!m_verbose)
          return;

        using namespace std;
        va_list ap;
        va_start(ap, format);
        vfprintf(stderr, format, ap);
        va_end(ap);
      }

      void
      Bootloader::title(const char* str) const
      {
        print("\n%s\n", str);
        print("------------------------------\n");
      }

      void
      Bootloader::reset(void)
      {
        print("\nResetting Device...");

        m_frame.setId(PKT_ID_RESET);
        m_frame.setPayloadSize(0);
        if (!m_itf->sendFrame(m_frame))
          throw std::runtime_error("failed to reset device");

        print(" OK\n");
      }

      void
      Bootloader::getFlashInfo(void)
      {
        m_frame.setId(PKT_ID_BOOT_FLASH_INFO);
        m_frame.setPayloadSize(0);
        if (!m_itf->sendFrame(m_frame))
          throw std::runtime_error("failed to retrieve flash info");

        m_frame.get(m_flash_size, 0);
        print("%-20s: %u\n", "Flash Size", m_flash_size);
        m_frame.get(m_page_size, 4);
        print("%-20s: %u\n", "Flash Page Size", m_page_size);
      }
    }
  }
}
