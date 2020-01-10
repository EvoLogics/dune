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
#include <xiapi_dng_store.h>
#else
#include <m3api/xiapi_dng_store.h>
#endif

// Local headers.
#include "Parser.hpp"

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
      unsigned int exposure;
      unsigned int data_format;
      fp32_t frame_rate;
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

      //! Camera ID.
      uint16_t m_id;
      unsigned int m_id_bm;

      //! Destination log folder.
      Path m_log_dir;

      //! Command Parser.
      Parser m_parser;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        xiH(NULL),
        m_sock(NULL),
        m_id(0),
        m_id_bm(0),
        m_parser(this)
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

        param("Exposure", m_args.exposure)
        .defaultValue("10")
        .units(Units::Millisecond)
        .description("Exposure time for the camera, 0 for auto");

        param("Data Format", m_args.data_format)
        .defaultValue("6")
        .description("Data format to use for image output, see XI_IMG_FORMAT");

        param("Frame Rate", m_args.frame_rate)
        .defaultValue("0")
        .description("Limit camera frame rate, 0 for max available");
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if (paramChanged(m_args.exposure))
          setExposure(m_args.exposure);

        if (paramChanged(m_args.data_format))
          setExposure(m_args.data_format);
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
        //! Bitmask L [1][2][3][4][5][6][_][_]  [7][8][9][10][11][12][_][_] H
        m_id_bm = 1 << (m_id - 1 + (m_id > c_max_id / 2 ? 2 : 0));
        debug("Camera Module ID bitmask is: %u", m_id_bm);

        try
        {
          inf("Opening the camera...");
          XICE(xiOpenDevice(0, &xiH));
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
          setDataFormat(m_args.data_format);
          setExposure(m_args.exposure);
          std::string sys_name = getSystemName();
          XICE(xiSetParamString(xiH, XI_PRM_DEVICE_USER_ID, (void*)sys_name.c_str(), (DWORD)sizeof(sys_name)));

          memset(&image, 0, sizeof(image));
          image.size = sizeof(XI_IMG);
        }
        catch(...)
        {
          throw RestartNeeded("Failed to initialize the camera", 10);
        }

        m_log_dir = m_ctx.dir_log / "Photos";
        if (!m_log_dir.exists())
          m_log_dir.create();
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        if (xiH != NULL)
          XICE(xiCloseDevice(xiH));

        Memory::clear(m_sock);
      }

      bool
      checkId(unsigned int index)
      {
        return ((index & m_id_bm) != 0);
      }

      void
      setDataFormat(unsigned int df)
      {
        //! Use only raw formats.
        if (!(df == XI_RAW8 or df == XI_RAW16))
          return;
        inf("Setting data format to %u...", df);
        XICE(xiSetParamInt(xiH, XI_PRM_IMAGE_DATA_FORMAT, df));
      }

      void
      setExposure(unsigned int exposure)
      {
        if (xiH == NULL)
          return;
        if (exposure == 0)
        {
          inf("Activating AEAG...");
          XICE(xiSetParamInt(xiH, XI_PRM_AEAG, XI_ON));
        }
        else
        {
          inf("Setting exposure time to %ums...", exposure);
          XICE(xiSetParamInt(xiH, XI_PRM_EXPOSURE, exposure * 1000));
        }
      }

      void
      setLedParams(unsigned int pulsew, unsigned int dimming)
      {
        //stub
      }

      void
      setFrameRate(fp32_t fr)
      {
        //stub
      }

      //! report capture status

      //! Command syntax:
      //! [S][cmd1][<-payload1->][;][cmd2][<-payload2->][/][any] = 4 + payload bytes
      //! Action:
      //! cmd: T - Trigger
      //! Option for above:
      //! cmd: F - Fire LED Flash
      //! Change parameters:
      //! cmd: E - Exposure
      //! cmd: D - Data Format
      //! cmd: R - Frame Rate
      //! cmd: L - LED Flash
      //! payload_T: [btm_id][top_id][n_frames]         = 6 bytes
      //! payload_F: [btm_id][top_id]                   = 4 bytes
      //! payload_E: [btm_id][top_id][exposure]         = 6 bytes
      //! payload_D: [btm_id][top_id][data_format]      = 6 bytes
      //! payload_R: [btm_id][top_id][fps]              = 6 bytes
      //! payload_L: [btm_id][top_id][pulse][dimming]   = 8 bytes

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

            if (rv < 8 or m_bfr[0] != 'S' or m_bfr[rv - 2] != '/')
              return;

            bool do_trigger = false;
            unsigned int n_frames = 0;
            bool do_flash = false;

            // Iterate discarding first and last received symbols.
            for (size_t i = 1; i < rv - 1; ++i)
            {
              if (m_parser.parse(m_bfr[i]))
              {
                spew("type: %c, id: %u, plc: %u", m_parser.getType(), m_parser.getId(), (unsigned)m_parser.getPayloadCount());

                // Check if the message addressed to this id.
                if (!checkId(m_parser.getId()))
                  continue;

                //TODO: check for payload length
                switch (m_parser.getType())
                {
                case 'T':
                  do_trigger = true;
                  n_frames = m_parser.getPayload();
                  spew("n_frames: %u", n_frames);
                  break;

                case 'F':
                  do_flash = true;
                  break;

                case 'E':
                  m_args.exposure = m_parser.getPayload();
                  spew("exposure: %u", m_args.exposure);
                  setExposure(m_args.exposure);
                  break;

                case 'D':
                  m_args.data_format = m_parser.getPayload();
                  spew("data_format: %u", m_args.data_format);
                  setDataFormat(m_args.data_format);
                  break;

                case 'R':
                  m_args.frame_rate = m_parser.getPayload() / 10.0;
                  spew("frame_rate: %0.2f", m_args.frame_rate);
                  setFrameRate(m_args.frame_rate);
                  break;

                case 'L':
                {
                  unsigned int pulsew = m_parser.getPayload(0);
                  unsigned int dimming = m_parser.getPayload(1);
                  spew("led params: %ums, %u%%", pulsew, dimming);
                  setLedParams(pulsew, dimming);
                }
                  break;

                default:
                  war("Got not supported command");
                }
              }
            }
            m_parser.reset();

            if (do_trigger and (n_frames > 0))
            {
              if (do_flash)
                xiSetParamInt(xiH, XI_PRM_GPO_MODE, XI_GPO_EXPOSURE_PULSE);

              getImages(n_frames);

              xiSetParamInt(xiH, XI_PRM_GPO_MODE, XI_GPO_OFF);
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

        double t_start = Clock::getSinceEpoch();
        for (uint images = 0; images < count; images++)
        {
          xiGetImage(xiH, 5000, &image); // getting next image from the camera opened

          XI_DNG_METADATA metadata;
          try
          {
            xidngFillMetadataFromCameraParams(xiH, &metadata);
          }
          catch(...)
          {
            war("Failed to fill metadata.");
          }
          //! Use own timestamp for consistency
          double now = Clock::getSinceEpoch();
          uint64_t t_sec = trunc(now);
          unsigned t_usec = trunc((now - t_sec) * 1000000);
          Time::BrokenDown bdt(t_sec);
          Path file = m_log_dir / String::str("%04u%02u%02u_%02u%02u%02u_%06u.dng", bdt.year,
              bdt.month, bdt.day, bdt.hour, bdt.minutes, bdt.seconds, t_usec);

          trace("Writing %s", file.c_str());
          XICE(xidngStore(file.c_str(), &image, &metadata));
        }
        inf("Acquired %u images in %0.3fs", count, Clock::getSinceEpoch() - t_start);
        inf("Stopping acquisition...");
        XICE(xiStopAcquisition(xiH));
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          readCmd(0.02);
          waitForMessages(0.02);
        }
      }
    };
  }
}

DUNE_TASK
