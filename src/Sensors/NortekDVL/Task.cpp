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

// ISO C++ 98 headers.
#include <cstring>
#include <algorithm>
#include <cstddef>

// DUNE headers.
#include <DUNE/DUNE.hpp>

// Local headers.
#include "Reader.hpp"

namespace Sensors
{
  //! Device driver for NMEA capable %GPS devices.
  namespace NortekDVL
  {
    using DUNE_NAMESPACES;

    //! Maximum number of initialization commands.
    static const unsigned c_max_init_cmds = 14;
    //! Timeout for waitReply() function.
    static const float c_wait_reply_tout = 4.0;
    static const unsigned c_pnorbt_fields = 10;
    static const unsigned c_pnors1_fields = 15;
    static const unsigned c_pnori1_fields = 7;
    static const unsigned c_pnorc1_fields = 17;
    //! Power on delay.
    static const double c_pwr_on_delay = 5.0;

    struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
      //! Order of sentences.
      std::vector<std::string> stn_order;
      //! Input timeout in seconds.
      float inp_tout;
      //! Initialization commands.
      std::string init_cmds[c_max_init_cmds];
      //! Initialization replies.
      std::string init_rpls[c_max_init_cmds];
      //! Power channels.
      std::vector<std::string> pwr_channels;
    };

    struct Task: public Tasks::Task
    {
      //! Serial port handle.
      IO::Handle* m_handle;
      IMC::GroundVelocity m_gvel;
      IMC::Temperature m_temp;
      IMC::Pressure m_prs;
      IMC::EulerAngles m_euler;
      //! Task arguments.
      Arguments m_args;
      double m_bdist, m_cell_len;
      std::string m_init_line;
      //! Reader thread.
      Reader* m_reader;

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx),
        m_handle(NULL),
        m_bdist(0),
        m_cell_len(0),
        m_reader(NULL)
      {
        // Define configuration parameters.
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("4800")
        .description("Serial port baud rate");

        param("Input Timeout", m_args.inp_tout)
        .units(Units::Second)
        .defaultValue("4.0")
        .minimumValue("0.0")
        .description("Input timeout");

        param("Power Channel - Names", m_args.pwr_channels)
        .defaultValue("")
        .description("Device's power channels");

        param("Sentence Order", m_args.stn_order)
        .defaultValue("")
        .description("Sentence order");

        for (unsigned i = 0; i < c_max_init_cmds; ++i)
        {
          std::string cmd_label = String::str("Initialization String %u - Command", i);
          param(cmd_label, m_args.init_cmds[i])
          .defaultValue("");

          std::string rpl_label = String::str("Initialization String %u - Reply", i);
          param(rpl_label, m_args.init_rpls[i])
          .defaultValue("");
        }

        m_euler.setSourceEntity(getEntityId());
        m_prs.setSourceEntity(getEntityId());
        m_temp.setSourceEntity(getEntityId());
        m_gvel.setSourceEntity(getEntityId());

        bind<IMC::DevDataText>(this);
        bind<IMC::IoEvent>(this);
      }

      void
      onResourceAcquisition(void)
      {
        if (m_args.pwr_channels.size() > 0)
        {
          IMC::PowerChannelControl pcc;
          pcc.op = IMC::PowerChannelControl::PCC_OP_TURN_ON;
          for (size_t i = 0; i < m_args.pwr_channels.size(); ++i)
          {
            pcc.name = m_args.pwr_channels[i];
            dispatch(pcc);
          }
        }

        Counter<double> timer(c_pwr_on_delay);
        while (!stopping() && !timer.overflow())
          waitForMessages(timer.getRemaining());

        try
        {
          if (!openSocket())
            m_handle = new SerialPort(m_args.uart_dev, m_args.uart_baud);

          m_reader = new Reader(this, m_handle);
          m_reader->start();
        }
        catch (...)
        {
          throw RestartNeeded(DTR(Status::getString(CODE_COM_ERROR)), 5);
        }
      }

      bool
      openSocket(void)
      {
        char addr[128] = {0};
        unsigned port = 0;

        if (std::sscanf(m_args.uart_dev.c_str(), "tcp://%[^:]:%u", addr, &port) != 2)
          return false;

        TCPSocket* sock = new TCPSocket;
        sock->connect(addr, port);
        m_handle = sock;
        return true;
      }

      void
      onResourceRelease(void)
      {
        if (m_reader != NULL)
        {
          m_reader->stopAndJoin();
          delete m_reader;
          m_reader = NULL;
        }

        Memory::clear(m_handle);
      }

      void
      onResourceInitialization(void)
      {
        for (unsigned i = 0; i < c_max_init_cmds; ++i)
        {
          if (m_args.init_cmds[i].empty())
            continue;

          std::string cmd = String::unescape(m_args.init_cmds[i]);
          m_handle->writeString(cmd.c_str());

          if (!m_args.init_rpls[i].empty())
          {
            std::string rpl = String::unescape(m_args.init_rpls[i]);
            if (!waitInitReply(rpl))
            {
              err("%s: %s", DTR("no reply to command"), m_args.init_cmds[i].c_str());
              throw std::runtime_error(DTR("failed to setup device"));
            }
          }
        }

        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      void
      consume(const IMC::DevDataText* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        spew("inp: %s", sanitize(msg->value).c_str());

        // if (getEntityState() == IMC::EntityState::ESTA_BOOT)
        //   m_init_line = msg->value;
        // else
          processSentence(msg->value);
      }

      void
      consume(const IMC::IoEvent* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        if (msg->type == IMC::IoEvent::IOV_TYPE_INPUT_ERROR)
          throw RestartNeeded(msg->error, 5);
      }

      //! Wait reply to initialization command.
      //! @param[in] stn string to compare.
      //! @return true on successful match, false otherwise.
      bool
      waitInitReply(const std::string& stn)
      {
        Counter<float> counter(c_wait_reply_tout);
        while (!stopping() && !counter.overflow())
        {
          waitForMessages(counter.getRemaining());
          if (m_init_line == stn)
          {
            m_init_line.clear();
            return true;
          }
        }

        return false;
      }

      //! Read decimal from input string.
      //! @param[in] str input string.
      //! @param[out] dst decimal.
      //! @return true if successful, false otherwise.
      template <typename T>
      bool
      readDecimal(const std::string& str, T& dst)
      {
        unsigned idx = 0;
        while (str[idx] == '0')
          ++idx;

        return castLexical(std::string(str, idx), dst);
      }

      //! Read number from input string.
      //! @param[in] str input string.
      //! @param[out] dst number.
      //! @return true if successful, false otherwise.
      template <typename T>
      bool
      readNumber(const std::string& str, T& dst)
      {
        return castLexical(str, dst);
      }

      //! Process sentence.
      //! @param[in] line line.
      void
      processSentence(const std::string& line)
      {
        // Discard leading noise.
        size_t sidx = 0;
        for (sidx = 0; sidx < line.size(); ++sidx)
        {
          if (line[sidx] == '$')
            break;
        }

        // Discard trailing noise.
        size_t eidx = 0;
        for (eidx = line.size() - 1; eidx > sidx; --eidx)
        {
          if (line[eidx] == '*')
            break;
        }

        if (sidx >= eidx)
          return;

        // Compute checksum.
        uint8_t ccsum = 0;
        for (size_t i = sidx + 1; i < eidx; ++i)
          ccsum ^= line[i];

        // Validate checksum.
        unsigned rcsum = 0;
        if (std::sscanf(&line[0] + eidx + 1, "%02X", &rcsum) != 1)
        {
          return;
        }

        // Split sentence
        std::vector<std::string> parts;
        String::split(line.substr(sidx + 1, eidx - sidx - 1), ",", parts);

        interpretSentence(parts);
      }

      //! Interpret given sentence.
      //! @param[in] parts vector of strings from sentence.
      void
      interpretSentence(std::vector<std::string>& parts)
      {
        if (parts[0] == "PNORBT")
        {
          interpretPNORBT(parts);
        }
        else if (parts[0] == "PNORS1")
        {
          interpretPNORS1(parts);
        }
        else if (parts[0] == "PNORI1")
        {
          interpretPNORI1(parts);
        }
        else if (parts[0] == "PNORC1")
        {
          interpretPNORC1(parts);
        }
      }

      void
      interpretPNORBT(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_pnorbt_fields)
        {
          war(DTR("invalid PNORBT sentence"));
          return;
        }

        readNumber(parts[8], m_bdist);
      }

      void
      interpretPNORS1(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_pnors1_fields)
        {
          war(DTR("invalid PNORS sentence"));
          return;
        }

        // readNumber(parts[7],  m_euler.psi);
        // readNumber(parts[9],  m_euler.theta);
        // readNumber(parts[11], m_euler.phi);
        // m_euler.psi   = Angles::normalizeRadian(Angles::radians(m_euler.psi));
        // m_euler.theta = Angles::normalizeRadian(Angles::radians(m_euler.theta));
        // m_euler.phi   = Angles::normalizeRadian(Angles::radians(m_euler.phi));
        // dispatch(m_euler);

        readNumber(parts[13],  m_prs.value);
        m_prs.value *= 100; // dBar -> hPa
        dispatch(m_prs);

        readNumber(parts[15],  m_temp.value);
        dispatch(m_temp);
      }

      void
      interpretPNORI1(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_pnori1_fields)
        {
          war(DTR("invalid PNORI1 sentence"));
          return;
        }

        readNumber(parts[6], m_cell_len);
      }

      void
      interpretPNORC1(const std::vector<std::string>& parts)
      {
        if (parts.size() < c_pnorc1_fields)
        {
          war(DTR("invalid PNORC1 sentence"));
          return;
        }

        double pos;
        readNumber(parts[4], pos);

        if (pos <= m_bdist && pos >= m_bdist - m_cell_len)
        {
          readNumber(parts[9],  m_gvel.x);
          readNumber(parts[10], m_gvel.y);
          readNumber(parts[11], m_gvel.z);
          dispatch(m_gvel);

          inf("vel %f, %f, %f", m_gvel.x, m_gvel.y, m_gvel.z);
        }
      }

      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
