//***************************************************************************
// Copyright 2007-2014 Universidade do Porto - Faculdade de Engenharia      *
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

#ifndef SENSORS_NORTEKDVL_READER_HPP_INCLUDED_
#define SENSORS_NORTEKDVL_READER_HPP_INCLUDED_

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <cstring>
#include <inttypes.h>
#include <algorithm>

#define MSTA_INIT               0
#define MSTA_CONF               1
#define MSTA_ERROR              2
#define MSTA_SEEK_HDR           3
#define MSTA_SEEK_CACHED_HDR    4
#define MSTA_CACHE_HDR          5
#define MSTA_CACHE_DATA         6

#define HDR_SIZE 10

namespace Sensors
{
  namespace NortekDVL
  {
    using DUNE_NAMESPACES;

    //! Read buffer size.
    static const size_t c_read_buffer_size = 4096;
    static const char *c_control_seq = "K1W%!Q\r\n";

    class Reader: public Concurrency::Thread
    {
    public:
      struct NortekParam
      {
        std::string username, password;
        double rate, sndvel, salinity;
        double bt_range, v_range, pwr_level;
      };

      //! Constructor.
      //! @param[in] task parent task.
      //! @param[in] handle I/O handle.
      Reader(Tasks::Task* task, IO::Handle* handle, NortekParam &param):
        m_task(task),
        m_handle(handle),
        m_state(MSTA_INIT),
        m_param(param)
      {
      }

      uint8_t
      getState(void)
      {
        ScopedMutex m(m_closed_mutex);
        return m_state;
      }

      void
      reconfigure(NortekParam &param)
      {
        ScopedMutex m(m_closed_mutex);

        m_handle->writeString(c_control_seq);
        m_state = MSTA_CONF;
        m_param = param;
        m_conf_line = 0;
      }

    private:
      //! Parent task.
      Tasks::Task* m_task;
      //! I/O handle.
      IO::Handle* m_handle;
      //! Internal read buffer.
      char m_buffer[c_read_buffer_size];
      uint8_t m_cache[c_read_buffer_size];
      //! State
      uint8_t m_state, m_conf_line;
      size_t m_cached;
      NortekParam m_param;
      //! Current line.
      std::string m_line;
      Mutex m_closed_mutex;

      void
      dispatch(IMC::Message& msg)
      {
        msg.setDestination(m_task->getSystemId());
        msg.setDestinationEntity(m_task->getEntityId());
        m_task->dispatch(msg, DF_LOOP_BACK);
      }

      void
      auth(void)
      {
        ScopedMutex m(m_closed_mutex);

        if (m_line.rfind("Username: ") != std::string::npos)
        {
          m_line.clear();
          m_handle->writeString(m_param.username.c_str());
          m_handle->writeString("\n");
        }
        else if (m_line.rfind("Password: ") != std::string::npos)
        {
          m_line.clear();
          m_handle->writeString(m_param.password.c_str());
          m_handle->writeString("\n");
        }
        else if (m_line.rfind("Command Interface\r\n") != std::string::npos) {
          m_line.clear();
          m_conf_line = 0;
          m_handle->writeString(c_control_seq);
          m_state = MSTA_CONF;
        }
        else if (m_line.rfind("Login failed") != std::string::npos)
        {
          throw std::runtime_error("Login failed");
        }
      }

      void
      conf(void)
      {
        ScopedMutex m(m_closed_mutex);
        std::string str;

        if (m_line.rfind("OK\r\n") != std::string::npos)
        {
          m_line.clear();
          switch (m_conf_line++)
          {
            case 0:
              m_handle->writeString("MC\r\n");
              break;

            case 1:
              str = String::str("SETDVL,2,\"OFF\",\"INTSR\",%.1f,\"\",%.1f,%.1f\r\n",
                      m_param.rate, m_param.sndvel, m_param.salinity);
              m_handle->writeString(str.c_str());
              break;

            case 2:
              str = String::str("SETBT,%.2f,%.2f,4,0,21,%.1f,\"XYZ\"\r\n",
                      m_param.bt_range, m_param.v_range, m_param.pwr_level);
              m_handle->writeString(str.c_str());
              break;

            case 3:
              str = String::str("SETCURPROF,1,0.50,0.10,\"XYZ\",%f,0.000,%f,3,4,0\r\n",
                      m_param.pwr_level, m_param.v_range);
              m_handle->writeString(str.c_str());
              break;

            case 4:
              m_handle->writeString("START\r\n");
              break;

            default:
              m_state = MSTA_SEEK_HDR;
          }
        }
        else if (m_line.rfind("ERROR\r\n") != std::string::npos)
        {
          m_line.clear();
          m_handle->writeString("GETERROR\r\n");
          m_state = MSTA_ERROR;
        }
      }

      void
      read(void)
      {
        if (!Poll::poll(*m_handle, 1.0))
          return;

        size_t pos;
        size_t rv = m_handle->read(m_buffer, c_read_buffer_size);
        if (rv == 0)
          throw std::runtime_error(DTR("invalid read size"));

        if (m_state < MSTA_SEEK_HDR)
        {
          m_line.append(m_buffer, rv);
          if (m_line.size() > c_read_buffer_size)
            m_line.erase(0, m_line.size() - c_read_buffer_size);

          switch (m_state)
          {
            case MSTA_INIT:
              auth();
              break;

            case MSTA_CONF:
              conf();
              break;

            case MSTA_ERROR:
              if ((pos = m_line.find('\n')) != std::string::npos)
                throw std::runtime_error(m_line.substr(0, pos - 1));
              break;
          }
        }
        else
        {
          size_t len, i;

          pos = 0; // current position in m_buffer[]
          while (pos < rv)
          {
            switch (m_state)
            {
              case MSTA_SEEK_HDR: // seek header in input buffer
                for (; pos < rv; ++pos)
                  if ((uint8_t)m_buffer[pos] == 0xA5)
                  {
                    m_state = MSTA_CACHE_HDR;
                    m_cached = 0;
                    break;
                  }

                break;

              case MSTA_SEEK_CACHED_HDR: // seek header in cache
                for (i = 1; i < m_cached; ++i)
                  if ((uint8_t)m_cache[i] == 0xA5)
                  {
                    std::memmove(m_cache, m_cache + i, m_cached - i);
                    m_cached -= i;
                    m_state = MSTA_CACHE_HDR;
                    break;
                  }

                if (m_state != MSTA_CACHE_HDR && i == m_cached) // not found
                {
                  m_state = MSTA_SEEK_HDR;
                }

                break;

              case MSTA_CACHE_HDR:
                len = std::min(rv - pos, (size_t)HDR_SIZE - m_cached);
                std::memcpy(m_cache + m_cached, m_buffer + pos, len);
                m_cached += len;
                pos += len;

                if (m_cached == HDR_SIZE)
                {
                  uint16_t sum = m_cache[8] | m_cache[9] << 8;

                  if (m_cache[1] != HDR_SIZE || sum != checksum(m_cache, HDR_SIZE - 2))
                    m_state = MSTA_SEEK_CACHED_HDR;
                  else
                    m_state = MSTA_CACHE_DATA;
                }

                break;

              case MSTA_CACHE_DATA:
                size_t datalen = m_cache[4] | m_cache[5] << 8;

                len = std::min(rv - pos, (size_t)HDR_SIZE + datalen - m_cached);
                std::memcpy(m_cache + m_cached, m_buffer + pos, len);
                m_cached += len;
                pos += len;

                if (m_cached == HDR_SIZE + datalen)
                {
                  uint16_t sum = m_cache[6] | m_cache[7] << 8;

                  if (sum != checksum(m_cache + HDR_SIZE, datalen))
                    m_state = MSTA_SEEK_CACHED_HDR;
                  else
                  {
                    processFrame();
                    m_state = MSTA_SEEK_HDR;
                  }
                }

                break;
            }
          }
        }
      }

      void
      processFrame(void)
      {
        IMC::DevDataBinary data;
        data.value.assign(m_cache, m_cache + m_cached);
        dispatch(data);
      }

      uint16_t
      checksum(uint8_t *data, size_t len)
      {
        uint16_t rs = 0xB58C;
        size_t nshorts = len >> 1;
        len -= nshorts << 1;

        while (nshorts--)
        {
          rs += (uint16_t)data[0] | ((uint16_t)data[1] << 8);
          data += 2;
        }

        if (len)
          rs += ((uint16_t)data[0]) << 8;

        return rs;
      }

      void
      run(void)
      {
        while (!isStopping())
        {
          try
          {
            read();
          }
          catch (std::runtime_error& e)
          {
            IMC::IoEvent evt;
            evt.type = IMC::IoEvent::IOV_TYPE_INPUT_ERROR;
            evt.error = e.what();
            dispatch(evt);
            break;
          }
        }
      }
    };
  }
}

#endif
