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

    struct Task: public DUNE::Tasks::Task
    {
      //! Camera handle.
      HANDLE xiH;
      //! Image buffer.
      XI_IMG image;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        xiH(NULL)
      {
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
        try
        {
          inf("Opening the camera...");
          XICE(xiOpenDevice(0, &xiH));
        }
        catch(...)
        {
          throw RestartNeeded("Failed to connect to the camera", 10);
        }
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        try
        {
          inf("Setting exposure time to 10ms...");
          XICE(xiSetParamInt(xiH, XI_PRM_EXPOSURE, 10000));

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
      }

      void
      getImages(const uint& count)
      {
        inf("Starting acquisition...");
        XICE(xiStartAcquisition(xiH));

        for (int images = 0; images < count; images++)
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
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
