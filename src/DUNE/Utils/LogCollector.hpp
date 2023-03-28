//***************************************************************************
// Copyright 2007-2023 EvoLogics GmbH                                       *
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
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: Michael Purser                                                   *
//***************************************************************************

#ifndef DUNE_UTILS_LOG_COLLECTOR_HPP_INCLUDED_
#define DUNE_UTILS_LOG_COLLECTOR_HPP_INCLUDED_

// ISO C++ 98 headers.
#include <map>
#include <string>
#include <vector>

namespace DUNE
{
  namespace Utils
  {
    class LogCollector
    {
    public:
      //! Add a log type to the collector.
      //! @param[in] id ID of the error type.
      void
      add(const unsigned id);

      //! Set a particular log type.
      //! @param[in] id ID of the log type.
      //! @param[in] msg log msg for setting the log type.
      void
      set(const unsigned id, const std::string& msg="");

      //! Unset a partocular log type.
      //! @param[in] id ID of the log type.
      //! @param[in] msg log msg for unsetting the log type.
      void
      unset(const unsigned id, const std::string& msg="");

      //! Check if there is at least one log type that is set.
      //! @return true if at least one log type that is set, false otherwise.
      bool
      isSet();

      //! Check if a particular log type is set.
      //! @param[in] id ID of the log type.
      //! @return true if the specified log type is set, false otherwise.
      bool
      isSet(const unsigned id);

      //! Generate a vector of msgs of all log types that were set since last called.
      //! Empty msgs are ignored.
      //! @return vector of msgs.
      std::vector<std::string>
      setMsgs(const bool keep_msgs = false);

      //! Generate a vector of msgs of all log types that were unset since last called.
      //! Empty msgs are ignored.
      //! @return vector of msgs.
      std::vector<std::string>
      unsetMsgs(const bool keep_msgs = false);

    private:
      //! Structure representing a log type to keep track of.
      struct Log
      {
        bool is_set;
        bool is_updated;
        std::string msg;
      };

      //! Map containing the log types, their status (set/unset) and their associated msgs.
      std::map<unsigned, Log> m_logs;
    };
  }
}

#endif
