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

// Vendor headers.
#ifdef DUNE_OS_WINDOWS
#include <xiApi.h>
#else
#include <m3api/xiApi.h>
#endif

// Check error macro. It executes function. Print and throw error if result is not OK.
#define XICE(func) {XI_RETURN stat = (func); if (XI_OK!=stat) {err("Error:%d returned from function:"#func"",stat);throw "Error";}}

namespace Vision
{
  namespace XimeaCamera
  {
    using DUNE_NAMESPACES;

    static const uint16_t c_max_id = 12;

    struct Arguments
    {
      Address udp_maddr;
      uint16_t udp_port;
      uint16_t base_id;
    };

    struct Task: public DUNE::Tasks::Task
    {
      // Configuration parameters.
      Arguments m_args;

      //! Camera handle.
      HANDLE xiH;
      //! Image buffer.
      XI_IMG image;

      //! UDP socket.
      UDPSocket* m_sock;
      //! Buffer.
      uint8_t m_bfr[1024];

      //! Camera ID
      uint16_t m_id;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        xiH(NULL),
        m_sock(NULL),
        m_id(0)
      {
        param("UDP Communications -- Multicast Address", m_args.udp_maddr)
        .defaultValue("227.0.0.1")
        .description("UDP multicast address for communications");

        param("UDP Communications -- Port", m_args.udp_port)
        .defaultValue("22701")
        .description("UDP port for communications");

        param("Base ID", m_args.base_id)
        .defaultValue("0x8400")
        .description("Base ID to subtract and get Camera Module ID");
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
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
        m_id = getSystemId() - m_args.base_id;

        //! ID is limited by 12 = 6 top + 6 bottom
        if (m_id > c_max_id)
        {
          err("Uncompatible ID: %u, fallback to 1", m_id);
          m_id = 1;
        }

        inf("Camera Module ID is: %u (%s)", m_id, m_id > c_max_id / 2 ? "bottom" : "top");

        try
        {
          inf("Opening the camera...");
          //XICE(xiOpenDevice(0, &xiH));
        }
        catch(...)
        {
          throw RestartNeeded("Failed to connect to the camera", 10);
        }

        m_sock = new DUNE::Network::UDPSocket();
        m_sock->setMulticastTTL(1);
        m_sock->setMulticastLoop(true);
        m_sock->joinMulticastGroup(m_args.udp_maddr);
        m_sock->bind(m_args.udp_port);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        try
        {
          inf("Setting exposure time to 10ms...");
          //XICE(xiSetParamInt(xiH, XI_PRM_EXPOSURE, 10000));

          memset(&image, 0, sizeof(image));
          image.size = sizeof(XI_IMG);
        }
        catch(...)
        {
          throw RestartNeeded("Failed to initialize the camera", 10);
        }
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        if (xiH != NULL)
          XICE(xiCloseDevice(xiH));

        Memory::clear(m_sock);
      }

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

      //! Command syntax:
      //! [S][cmd][<-payload->][/][any] = 4 + payload bytes
      //! cmd: T - Trigger
      //! payload: [n_frames][btm_tr][top_tr][btm_fl][top_fl] = 10 bytes

      void
      readCmd(const double& timeout)
      {
        Address addr;
        uint16_t port;

        try
        {
          if (Poll::poll(*m_sock, timeout))
          {
            size_t rv = m_sock->read(m_bfr, sizeof(m_bfr), &addr, &port);
            spew("received %u bytes from %s:%u", (unsigned)rv, addr.c_str(), port);
            if (m_bfr[0] == 'S' and m_bfr[rv - 2] == '/')
            {
              if (m_bfr[1] == 'T' and rv >= 14)
              {
                unsigned int n_frames = byteFromHex(&m_bfr[2]);
                unsigned int ind_tr = wordFromHex(&m_bfr[4]);
                unsigned int ind_fl = wordFromHex(&m_bfr[8]);

                spew("n_frames: %u", n_frames);
                spew("ind_tr: %u", ind_tr);
                spew("ind_fl: %u", ind_fl);
              }
            }
          }
        }
        catch (std::runtime_error& e)
        {
          err("Read error: %s", e.what());
        }
      }

      void
      getImages(const uint& count)
      {
        inf("Starting acquisition...");
        XICE(xiStartAcquisition(xiH));

        for (uint images = 0; images < count; images++)
        {
          xiGetImage(xiH, 5000, &image); // getting next image from the camera opened
          unsigned char pixel = *(unsigned char*)image.bp;
          inf("Image %d (%dx%d) received from camera. First pixel value: %d", images, (int)image.width, (int)image.height, pixel);
        }

        inf("Stopping acquisition...");
        XICE(xiStopAcquisition(xiH));
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          readCmd(0.1);
          waitForMessages(0.1);
        }
      }
    };
  }
}

DUNE_TASK
