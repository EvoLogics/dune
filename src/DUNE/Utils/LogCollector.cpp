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

// Local headers.
#include "LogCollector.hpp"

namespace DUNE
{
  namespace Utils
  {
    void
    LogCollector::add(const unsigned id)
    {
      m_logs.insert({id, {false, false, ""}});
    }

    void
    LogCollector::set(const unsigned id, const std::string& msg)
    {
      if (m_logs.count(id) == 0)
        add(id);

      if (!m_logs.at(id).is_set || msg != m_logs.at(id).msg)
      {
        m_logs.at(id).is_set = true;
        m_logs.at(id).is_updated = true;
        m_logs.at(id).msg = msg;
      }
    }

    void
    LogCollector::unset(const unsigned id, const std::string& msg)
    {
      if (m_logs.count(id) == 0)
        add(id);

      if (m_logs.at(id).is_set || msg != m_logs.at(id).msg)
      {
        m_logs.at(id).is_set = false;
        m_logs.at(id).is_updated = true;
        m_logs.at(id).msg = msg;
      }
    }

    bool
    LogCollector::isSet()
    {
      std::map<unsigned, Log>::iterator it;
      for (it = m_logs.begin(); it != m_logs.end(); it++)
      {
        if (it->second.is_set)
          return true;
      }
      return false;
    }

    bool
    LogCollector::isSet(const unsigned id)
    {
      return m_logs.at(id).is_set;
    }

    std::vector<std::string>
    LogCollector::setMsgs(const bool keep_msgs)
    {
      std::vector<std::string> str;
      std::map<unsigned, Log>::iterator it;
      for (it = m_logs.begin(); it != m_logs.end(); it++)
      {
        if ((it->second.is_set) && (it->second.is_updated) && (!it->second.msg.empty()))
        {
          str.emplace_back(it->second.msg);
          if (!keep_msgs)
          {
            it->second.is_updated = false;
          }
        }
      }
      return str;
    }

    std::vector<std::string>
    LogCollector::unsetMsgs(const bool keep_msgs)
    {
      std::vector<std::string> str;
      std::map<unsigned, Log>::iterator it;
      for (it = m_logs.begin(); it != m_logs.end(); it++)
      {
        if ((!it->second.is_set) && (it->second.is_updated) && (!it->second.msg.empty()))
        {
          str.emplace_back(it->second.msg);
          if (!keep_msgs)
          {
            it->second.is_updated = false;
          }
        }
      }
      return str;
    }
  }
}
