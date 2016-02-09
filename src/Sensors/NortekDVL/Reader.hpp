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
        m_state(0),
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
        m_state = 1; // CONF
        m_param = param;
      }

    private:
      //! Parent task.
      Tasks::Task* m_task;
      //! I/O handle.
      IO::Handle* m_handle;
      //! Internal read buffer.
      char m_buffer[c_read_buffer_size];
      //! State
      uint8_t m_state, m_conf_line;
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
          m_state = 1; // CONF
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
              str = String::str("SETDVL,0,\"OFF\",\"INTSR\",%.1f,\"\",%.1f,%.1f\r\n",
                      m_param.rate, m_param.sndvel, m_param.salinity);
              m_handle->writeString(str.c_str());
              break;

            case 2:
              str = String::str("SETBT,%.2f,%.2f,4,0,307,%.1f,\"XYZ\"\r\n",
                      m_param.bt_range, m_param.v_range, m_param.pwr_level);
              m_handle->writeString(str.c_str());
              break;

            case 3:
              m_handle->writeString("START\r\n");
              break;

            default:
              m_state = 2; // CAPTURE
          }
        }
        else if (m_line.rfind("ERROR\r\n") != std::string::npos)
        {
          m_line.clear();
          m_handle->writeString("GETERROR\r\n");
          m_state = 3; // ERROR
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

        switch (m_state)
        {
          case 0: // INIT
            m_line.append(m_buffer, rv);
            if (m_line.size() > c_read_buffer_size)
              m_line.erase(0, m_line.size() - c_read_buffer_size);

            auth();
            break;

          case 1: // CONF
            m_line.append(m_buffer, rv);
            if (m_line.size() > c_read_buffer_size)
              m_line.erase(0, m_line.size() - c_read_buffer_size);

            conf();
            break;

          case 2: // CAPTURE
            m_line.append(m_buffer, rv);
            if (m_line.size() > c_read_buffer_size)
              m_line.erase(0, m_line.size() - c_read_buffer_size);

            while ((pos = m_line.find('\n')) != std::string::npos)
            {
              IMC::DevDataText line;
              line.value = m_line.substr(0, pos - 1);
              m_line.erase(0, pos + 1);
              dispatch(line);
            }
            break;

          case 3: // ERROR
          default:
            m_line.append(m_buffer, rv);
            if (m_line.size() > c_read_buffer_size)
              m_line.erase(0, m_line.size() - c_read_buffer_size);

            if ((pos = m_line.find('\n')) != std::string::npos)
              throw std::runtime_error(m_line.substr(0, pos - 1));

            break;
        }
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
