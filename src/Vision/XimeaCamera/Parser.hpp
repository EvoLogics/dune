//***************************************************************************
// Copyright 2007-2019 EvoLogics GmbH                                       *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Viktor Voronin                                                   *
//***************************************************************************

#ifndef VISION_XIMEACAMERA_PARSER_HPP_INCLUDED_
#define VISION_XIMEACAMERA_PARSER_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Vision
{
  namespace XimeaCamera
  {
    using DUNE_NAMESPACES;

    static const uint16_t c_max_pl = 2;

    class Parser
    {
    public:
      enum State
      {
        ST_TYPE,
        ST_ID,
        ST_PAYLOAD
      };

      Parser(Tasks::Task* parent):
        m_parent(parent),
        m_state(ST_TYPE),
        m_type(),
        m_id(0),
        m_payload_index(0)
      {
        m_bfr.resize(16, 0);
        m_bfr.clear();
        m_payload.resize(c_max_pl, 0);
        m_payload.clear();
      }

      ~Parser(void)
      { }

      //! Parse one byte of data.
      //! @param[in] byte data byte.
      //! @return true when the command is parsed.
      bool
      parse(uint8_t byte)
      {
        switch (m_state)
        {

          case ST_TYPE:
            m_type = byte;
            m_payload_index = 0;
            m_state = ST_ID;
            break;

          case ST_ID:
          	m_bfr.push_back(byte);
            if (m_bfr.size() == 4)
            {
              m_id = wordFromHex(&m_bfr[0]);
              m_bfr.clear();
              // switch to next state
              m_state = ST_PAYLOAD;
            }
            break;

          case ST_PAYLOAD:
            if (byte == ';' or byte == '/')
            {
              m_state = ST_TYPE;
              return true;
            }
          	m_bfr.push_back(byte);
            if (m_bfr.size() == 2)
            {
              if (m_payload_index < c_max_pl)
              {
                m_payload[m_payload_index] = byteFromHex(&m_bfr[0]);
                ++m_payload_index;
              }
              else
              {
                m_parent->war("Got unexpected payload");
              }
              m_bfr.clear();
            }
            break;

        }

        return false;
      }

      void
      reset(void)
      {
        m_state = ST_TYPE;
        m_payload_index = 0;
      }

      State
      getState(void) const
      {
        return m_state;
      }

      char
      getType(void)
      {
        return m_type;
      }

      uint16_t
      getId(void)
      {
        return m_id;
      }

      unsigned int
      getPayload(size_t index = 0)
      {
        return m_payload[index];
      }

      size_t
      getPayloadCount(void)
      {
        return m_payload_index;
      }

    private:
      //! Parent task.
      Tasks::Task* m_parent;
      //! Current parser state.
      State m_state;
      //! Read buffer.
      std::vector<uint8_t> m_bfr;
      //! Parsed data.
      char m_type;
      uint16_t m_id;
      std::vector<unsigned int> m_payload;
      //! Payload index
      size_t m_payload_index;

      unsigned int
      byteFromHex(uint8_t* val)
      {
        unsigned int result;
        std::stringstream ss;
        ss << std::hex;
        for (uint i = 0; i < 2; i++)
            ss << *(val + i);
        ss >> result;
        return result;
      }

      unsigned int
      wordFromHex(uint8_t* val)
      {
        unsigned int result;
        std::stringstream ss;
        ss << std::hex;
        for (uint i = 0; i < 4; i++)
            ss << *(val + i);
        ss >> result;
        return result;
      }
    };
  }
}

#endif
