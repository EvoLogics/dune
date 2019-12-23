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

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Actuators
{
  namespace EvoLamp
  {
    using DUNE_NAMESPACES;

    struct Arguments
    {
      // IO device.
      std::string io_dev;
      // Serial port baud rate.
      unsigned io_baud;
      // Pulse duration.
      unsigned pwm_all;
      // Dimming value.
      unsigned dac_all;
    };

    struct Task: public DUNE::Tasks::Task
    {
      // Maximum DAC dimming value.
      static const unsigned c_dac_max = 4095;
      //! Maximum UDP data packet size.
      static const unsigned c_bfr_size = 1024;
      // Task parameters.
      Arguments m_args;
      // IO device handle.
      IO::Handle* m_iohandle;
      // Buffer.
      uint8_t* m_bfr;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_iohandle(NULL),
        m_bfr(NULL)
      {
        param("IO Device", m_args.io_dev)
        .defaultValue("")
        .description("Device to connect: tcp socket or serial port");

        param("Baud Rate", m_args.io_baud)
        .defaultValue("115200")
        .description("Baud rate for serial connection");

        param("Pulse Duration", m_args.pwm_all)
        .units(Units::Millisecond)
        .defaultValue("20")
        .description("Pulse duration (duty cycle, in microseconds)");

        param("Dimming Value", m_args.dac_all)
        .units(Units::Percentage)
        .minimumValue("0")
        .maximumValue("100")
        .defaultValue("50")
        .description("Analogue dimming value");
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if (m_iohandle == NULL)
          return;

        if (paramChanged(m_args.io_dev) || paramChanged(m_args.io_baud))
          throw RestartNeeded(DTR("restarting to change IO parameters"), 1);

        if (paramChanged(m_args.pwm_all))
          setPwmAll(m_args.pwm_all);

        if (paramChanged(m_args.dac_all))
          setDacAll(m_args.dac_all);
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        m_bfr = new uint8_t[c_bfr_size];

        try
        {
          if (!tryTcpSocket())
          {
            trace("opening %s@%u", m_args.io_dev.c_str(), m_args.io_baud);
            m_iohandle = new SerialPort(m_args.io_dev, m_args.io_baud);
          }
          m_iohandle->flush();
        }
        catch (...)
        {
          throw RestartNeeded("failed to connect to device", 5);
        }
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        setPwmAll(m_args.pwm_all);
        setDacAll(m_args.dac_all);
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        Memory::clear(m_bfr);
        Memory::clear(m_iohandle);
      }

      bool
      tryTcpSocket(void)
      {
        char addr[128] = {0};
        unsigned port = 0;

        if (std::sscanf(m_args.io_dev.c_str(), "tcp://%[^:]:%u", addr, &port) != 2)
          return false;

        trace("connecting to %s:%u", addr, port);

        TCPSocket* sock = new TCPSocket;
        sock->connect(addr, port);
        m_iohandle = sock;
        return true;
      }

      void
      setPwmAll(const unsigned duration)
      {
        std::string cmd = String::str("PWM.ALL=%u", duration * 1000);
        sendCmd(cmd);
      }

      void
      setDacAll(const unsigned dimming)
      {
        std::string cmd = String::str("DAC.ALL=%u", (unsigned)trunc(dimming / 100.0 * c_dac_max));
        sendCmd(cmd);
      }

      void
      sendCmd(const std::string& command)
      {
        std::string reply = (String::str(">%s\n", command.c_str()));
        try
        {
          m_iohandle->write(reply.c_str(), reply.size());
          inf("Sent Cmd: %s", command.c_str());
        }
        catch(std::exception& e)
        {
          throw RestartNeeded(e.what(), 5);
        }
      }

      bool
      readData(double timeout = 1.0)
      {
        if (m_iohandle == NULL)
          return false;

        if (!Poll::poll(*m_iohandle, timeout))
          return false;

        size_t rv = m_iohandle->read(m_bfr, c_bfr_size);

        std::string msg((char*)m_bfr, rv - 1);
        msg = String::rtrim(msg);
        spew("rcvd: %s", msg.c_str());

        return true;
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          consumeMessages();
          readData(0.1);
        }
      }
    };
  }
}

DUNE_TASK
