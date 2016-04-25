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

const double deg2rad = M_PI / 180.0;

namespace Sensors
{
  //! Device driver for NMEA capable %GPS devices.
  namespace NortekDVL
  {
    using DUNE_NAMESPACES;

    static const double c_pwr_on_delay = 5.0;
    static const double c_init_tout = 10.0;

    struct Arguments
    {
      //! Serial port device.
      std::string uart_dev;
      //! Serial port baud rate.
      unsigned uart_baud;
      //! Input timeout in seconds.
      float inp_tout;
      //! Power channels.
      std::vector<std::string> pwr_channels;
      //! Rotation angles of DVL-frame
      std::vector<double> rotation;

      Reader::NortekParam params;
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
      std::string m_init_line;
      //! Reader thread.
      Reader* m_reader;
      double m_dcm[9];

      Task(const std::string& name, Tasks::Context& ctx):
        Tasks::Task(name, ctx),
        m_handle(NULL),
        m_reader(NULL)
      {
        updateDCM(0, 0, 0);

        // Define configuration parameters.
        param("Serial Port - Device", m_args.uart_dev)
        .defaultValue("")
        .description("Serial port device used to communicate with the sensor");

        param("Serial Port - Baud Rate", m_args.uart_baud)
        .defaultValue("4800")
        .description("Serial port baud rate");

        param("Input Timeout", m_args.inp_tout)
        .units(Units::Second)
        .defaultValue("5.0")
        .minimumValue("0.0")
        .description("Input timeout");

        param("Power Channel - Names", m_args.pwr_channels)
        .defaultValue("")
        .description("Device's power channels");

        param("Username", m_args.params.username)
        .defaultValue("nortek")
        .description("User name to athenticate command interface");

        param("Password", m_args.params.password)
        .defaultValue("")
        .description("Password to athenticate command interface");

        param("Input Rate", m_args.params.rate)
        .defaultValue("4.0")
        .minimumValue("0.0")
        .description("Input rate");

        param("Sound Velocity", m_args.params.sndvel)
        .defaultValue("0.0")
        .description("Sound velocity");

        param("Salinity", m_args.params.salinity)
        .defaultValue("0.0")
        .description("Salinity");

        param("Bottom-Track Range", m_args.params.bt_range)
        .defaultValue("30.0")
        .description("Bottom-track range");

        param("Velocity Range", m_args.params.v_range)
        .defaultValue("5.0")
        .description("Velocity range");

        param("Power Level", m_args.params.pwr_level)
        .defaultValue("-20.0")
        .description("Power level");

        param("Rotation", m_args.rotation)
        .defaultValue("0, 0, 0")
        .size(3)
        .description("Rotation angles of DVL-frame");

        m_euler.setSourceEntity(getEntityId());
        m_prs.setSourceEntity(getEntityId());
        m_temp.setSourceEntity(getEntityId());
        m_gvel.setSourceEntity(getEntityId());

        bind<IMC::DevDataBinary>(this);
        bind<IMC::IoEvent>(this);
      }

      void
      onUpdateParameters(void)
      {
        if (m_reader)
        {
          if (paramChanged(m_args.params.username) ||
              paramChanged(m_args.params.password) ||
              paramChanged(m_args.params.rate) ||
              paramChanged(m_args.params.sndvel) ||
              paramChanged(m_args.params.salinity) ||
              paramChanged(m_args.params.bt_range) ||
              paramChanged(m_args.params.v_range) ||
              paramChanged(m_args.params.pwr_level))
            m_reader->reconfigure(m_args.params);
        }

        if (paramChanged(m_args.rotation))
            updateDCM(m_args.rotation[0] * deg2rad,
                      m_args.rotation[1] * deg2rad,
                      m_args.rotation[2] * deg2rad);
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

          m_reader = new Reader(this, m_handle, m_args.params);
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
        bool ok = false;
        Counter<float> counter(c_init_tout);
        while (!stopping() && !counter.overflow())
        {
          waitForMessages(counter.getRemaining());
          if (m_reader->getState() >= MSTA_SEEK_HDR) {
            ok = true;
            break;
          }
        }

        if (!ok)
          throw std::runtime_error(DTR("failed to setup device"));

        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
      }

      void
      consume(const IMC::DevDataBinary* msg)
      {
        if (msg->getDestination() != getSystemId())
          return;

        if (msg->getDestinationEntity() != getEntityId())
          return;

        processFrame(&msg->value[0], msg->value.size());
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

      void
      processFrame(const char *data, size_t len)
      {
        switch (data[2])
        {
          case 0x1B:
            processBottomTrack(data, len);
            break;

          case 0x16:
            processAverageData(data, len);
            break;

          default:
            inf("not supported: %" PRIx8, data[2]);
        }
      }

      void
      processBottomTrack(const char *data, size_t len)
      {
        uint32_t status;
        std::memcpy(&status, data + HDR_SIZE + 20, sizeof(uint32_t));

        float vx, vy, vz;
        std::memcpy(&vx, data + HDR_SIZE + 132, sizeof(float));
        std::memcpy(&vy, data + HDR_SIZE + 136, sizeof(float));
        std::memcpy(&vz, data + HDR_SIZE + 140, sizeof(float));

        m_gvel.x = vx * m_dcm[0] + vy * m_dcm[1] + vz * m_dcm[2];
        m_gvel.y = vx * m_dcm[3] + vy * m_dcm[4] + vz * m_dcm[5];
        m_gvel.z = vx * m_dcm[6] + vy * m_dcm[7] + vz * m_dcm[8];
        m_gvel.validity = (status >> 12) & 7;

        if (((status >> 12) & 0x07) == 0x07)
          dispatch(m_gvel);

        float prs;
        std::memcpy(&prs, data + HDR_SIZE + 32, sizeof(float));
        m_prs.value = prs * 1000;
        dispatch(m_prs);

        float temp;
        std::memcpy(&temp, data + HDR_SIZE + 28, sizeof(float));
        m_temp.value = temp;
        dispatch(m_temp);

        spew("vel: (%.2f, %.2f, %.2f), prs: %.2f, temp: %.1f, valid_bits: %d%d%d",
                m_gvel.x, m_gvel.y, m_gvel.z,
                prs * 10, temp,
                (status >> 12) & 1, (status >> 13) & 1, (status >> 14) & 1);

        (void)len;
      }

      void
      processAverageData(const char *data, size_t len)
      {
        // uint16_t yaw;
        // int16_t roll, pitch;
        // std::memcpy(&roll,  data + HDR_SIZE + 24, sizeof(uint16_t));
        // std::memcpy(&pitch, data + HDR_SIZE + 26, sizeof(int16_t));
        // std::memcpy(&yaw,   data + HDR_SIZE + 28, sizeof(int16_t));

        // spew("rpy: (%.2f , %.2f, %.2f)",
        //         (float)roll, (float)pitch, (float)yaw);
        (void)data;
        (void)len;
      }

      void
      updateDCM(double roll, double pitch, double yaw)
      {
        double cr = cos(roll), cp = cos(pitch), cy = cos(yaw);
        double sr = sin(roll), sp = sin(pitch), sy = sin(yaw);

        m_dcm[0] = cp * cy;
        m_dcm[1] = sr * sp * cy - cr * sy;
        m_dcm[2] = cr * sp * cy + sr * sy;

        m_dcm[3] = cp * sy;
        m_dcm[4] = sr * sp * sy + cr * cy;
        m_dcm[5] = cr * sp * sy - sr * cy;

        m_dcm[6] = -sp;
        m_dcm[7] = sr * cp;
        m_dcm[8] = cr * cp;
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
