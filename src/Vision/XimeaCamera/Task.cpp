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

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        xiH(NULL),
        m_sock(NULL),
        m_id(0),
        m_id_bm(0)
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
          //XICE(xiSetParamFloat(xiH, XI_PRM_LENS_FOCAL_LENGTH XI_PRM_INFO_MIN, 8));
          //XICE(xiSetParamFloat(xiH, XI_PRM_LENS_FOCAL_LENGTH XI_PRM_INFO_MAX, 8));
          //XICE(xiSetParamFloat(xiH, XI_PRM_LENS_APERTURE_VALUE, 6));

          memset(&image, 0, sizeof(image));
          image.size = sizeof(XI_IMG);
        }
        catch(...)
        {
          throw RestartNeeded("Failed to initialize the camera", 10);
        }

        m_log_dir = m_ctx.dir_log / "Photos";
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
      //! cmd: E - Exposure
      //! cmd: D - Data Format
      //! payload_T: [btm_id][top_id][n_frames][btm_fl][top_fl] = 10 bytes
      //! payload_E: [btm_id][top_id][exposure] = 6 bytes
      //! payload_D: [btm_id][top_id][data_format] = 6 bytes

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

            unsigned int ind_id = wordFromHex(&m_bfr[2]);
            spew("ind_id: %u", ind_id);

            if (not checkId(ind_id))
              return;

            if (m_bfr[1] == 'T' and rv >= 14)
            {
              unsigned int n_frames = byteFromHex(&m_bfr[6]);
              unsigned int ind_fl = wordFromHex(&m_bfr[8]);

              spew("n_frames: %u", n_frames);
              spew("ind_fl: %u", ind_fl);

              if (n_frames > 0)
                getImages(n_frames);
            }
            else if (m_bfr[1] == 'E' and rv >= 10)
            {
              m_args.exposure = byteFromHex(&m_bfr[6]);
              spew("exposure: %u", m_args.exposure);
              setExposure(m_args.exposure);
            }
            else if (m_bfr[1] == 'D' and rv >= 10)
            {
              m_args.data_format = byteFromHex(&m_bfr[6]);
              spew("data_format: %u", m_args.data_format);
              setDataFormat(m_args.data_format);
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
          unsigned char pixel = *(unsigned char*)image.bp;
          inf("Image %d (%dx%d) received from camera. First pixel value: %d", images, (int)image.width, (int)image.height, pixel);

          inf("tsSec: %u, tsUSec: %u", image.tsSec, image.tsUSec);

          inf("Writing DNG...");
          XI_DNG_METADATA metadata;
          try
          {
            xidngFillMetadataFromCameraParams(xiH, &metadata);
            inf("%u", metadata.acqTimeMonth);
          }
          catch(...)
          {
            war("Failed to fill metadata.");
          }
          Path file = m_log_dir / String::str("%04u%02u%02u_%02u%02u%02u_%06u.dng", metadata.acqTimeYear,
              metadata.acqTimeMonth, metadata.acqTimeDay, metadata.acqTimeHour,
              metadata.acqTimeMinute, metadata.acqTimeSecond, image.tsUSec);
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
