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
#include "ErrorAggregator.hpp"

namespace DUNE
{
  namespace Utils
  {
    ErrorAggregator::ErrorAggregator(DUNE::Tasks::Task* task):
      m_task(task)
    {
    }

    void
    ErrorAggregator::setError(const unsigned id, const std::string& msg)
    {
      if (m_errors.count(id) == 0)
        add(id);

      if (!m_errors.at(id).is_set || msg != m_errors.at(id).msg)  
        m_task->err("%s", msg.c_str());

      m_errors.at(id).is_set = true;
      m_errors.at(id).msg = msg;
    }

    void
    ErrorAggregator::unsetError(const unsigned id, const std::string& msg)
    {
      if (m_errors.count(id) == 0)
        add(id);

      if ((m_errors.at(id).is_set || msg != m_errors.at(id).msg) && !msg.empty())  
        m_task->inf("%s", msg.c_str());

      m_errors.at(id).is_set = false;
      m_errors.at(id).msg = msg;
    }

    bool
    ErrorAggregator::isSet() const
    {
      bool is_set = false;

      for (auto it = m_errors.begin(); it != m_errors.end(); it++)
        is_set |= it->second.is_set;

      return is_set;
    }

    bool
    ErrorAggregator::isSet(const unsigned id) const
    {
      return m_errors.at(id).is_set;
    }

    void
    ErrorAggregator::add(const unsigned id)
    {
      m_errors.insert({id, {false, ""}});
    }
  }
}
