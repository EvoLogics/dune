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

#ifndef DUNE_UTILS_ERROR_AGGREGATOR_HPP_INCLUDED_
#define DUNE_UTILS_ERROR_AGGREGATOR_HPP_INCLUDED_

// STL headers.
#include <map>
#include <string>

// DUNE headers.
#include <DUNE/Tasks/Task.hpp>

namespace DUNE
{
  namespace Utils
  {
    class ErrorAggregator
    {
    public:
      ErrorAggregator(DUNE::Tasks::Task* task);

      //! Set a particular error, and log the message (as an error) either if the error was previously not set,
      //! or the message changed.
      //! @param[in] id ID of the error.
      //! @param[in] msg error message.
      void
      setError(const unsigned id, const std::string& msg);

      //! Unset a particular error, and log the message (as an inf) either if the error was previously set,
      //! or the message changed. If the message is empty, it will not be logged.
      //! @param[in] id ID of the error.
      //! @param[in] msg message.
      void
      unsetError(const unsigned id, const std::string& msg="");

      //! Check if there is at least one error that is set.
      //! @return true if at least one error is set, false otherwise.
      bool
      isSet() const;

      //! Check if a particular error is set.
      //! @param[in] id ID of the error.
      //! @return true if the specified error is set, false otherwise.
      bool
      isSet(const unsigned id) const;

    private:
      //! Structure representing an error to keep track of.
      struct Error
      {
        bool is_set;
        std::string msg;
      };

      //! Add an error to the aggregator (not set and with an empty message).
      //! @param[in] id ID of the error.
      void
      add(const unsigned id);

      //! Map containing the errors, their status (set/unset) and their associated msgs.
      std::map<unsigned, Error> m_errors;
      //! Pointer to the task that is using the error aggregator.
      DUNE::Tasks::Task* m_task;
    };
  }
}

#endif
