// Copyright (c) 2016 Shaun Edwards
// Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific (ROS 2 compatibility)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef PACKML_SM__COMMON_HPP_
#define PACKML_SM__COMMON_HPP_

#include <ostream>
#include <string>
#include <type_traits>

namespace packml_sm
{


/* This magic function allows iostream (i.e. ROS_##_STREAM) macros to print out
* enumerations
* see: http://stackoverflow.com/questions/11421432/how-can-i-output-the-value-of-an-enum-class-in-c11
*/
template<typename T>
std::ostream & operator<<(
  typename std::enable_if<std::is_enum<T>::value,
  std::ostream>::type & stream, const T & e)
{
  return stream << static_cast<typename std::underlying_type<T>::type>(e);
}


/* Super states that encapsulate multiple substates with a common transition
* Not explicitly used in the standard but helpful for constructing the state
* machine.
*/
enum class SuperState {
  ABORTABLE = 0,
  STOPPABLE = 1
};

inline std::string to_string(const SuperState& state)
{
  switch (state) {
    case SuperState::ABORTABLE: return "ABORTABLE";
    case SuperState::STOPPABLE: return "STOPPABLE";
  }
  return std::to_string(static_cast<typename std::underlying_type<SuperState>::type>(state));
}

inline std::ostream& operator<< (std::ostream& os, SuperState state)
{
  return os << to_string(state);
}


// Aligned with State.msg enum
enum class State
{
  UNDEFINED    = 0,
  CLEARING     = 1,
  STOPPED      = 2,
  STARTING     = 3,
  IDLE         = 4,
  SUSPENDED    = 5,
  EXECUTE      = 6,
  STOPPING     = 7,
  ABORTING     = 8,
  ABORTED      = 9,
  HOLDING      = 10,
  HELD         = 11,
  UNHOLDING    = 12,
  SUSPENDING   = 13,
  UNSUSPENDING = 14,
  RESETTING    = 15,
  COMPLETING   = 16,
  COMPLETE     = 17,
};

inline std::string to_string(const State& state)
{
  switch (state)
  {
    case State::UNDEFINED:  return "UNDEFINED";
    case State::CLEARING:   return "CLEARING";
    case State::STOPPED:    return "STOPPED";
    case State::STARTING:   return "STARTING";
    case State::IDLE:       return "IDLE";
    case State::SUSPENDED:  return "SUSPENDED";
    case State::EXECUTE:    return "EXECUTE";
    case State::STOPPING:   return "STOPPING";
    case State::ABORTING:   return "ABORTING";
    case State::ABORTED:    return "ABORTED";
    case State::HOLDING:    return "HOLDING";
    case State::HELD:       return "HELD";
    case State::UNHOLDING:  return "UNHOLDING";
    case State::SUSPENDING: return "SUSPENDING";
    case State::UNSUSPENDING: return "UNSUSPENDING";
    case State::RESETTING:  return "RESETTING";
    case State::COMPLETING: return "COMPLETING";
    case State::COMPLETE:   return "COMPLETE";
  };
  return std::to_string(static_cast<typename std::underlying_type<State>::type>(state));
}

inline std::ostream& operator<< (std::ostream& os, State state)
{
  return os << to_string(state);
}

// Aligned with Mode.msg
enum class ModeType
{
  UNDEFINED   = 0,
  PRODUCTION  = 1,
  MAINTENANCE = 2,
  MANUAL      = 3
};

inline std::string to_string(const ModeType& mode)
{
  switch (mode) {
    case ModeType::UNDEFINED:   return "UNDEFINED";
    case ModeType::PRODUCTION:  return "PRODUCTION";
    case ModeType::MAINTENANCE: return "MAINTENANCE";
    case ModeType::MANUAL:      return "MANUAL";
  }
  if (mode > ModeType::MANUAL) {
    return "USER_DEFINED";
  }
  return std::to_string(static_cast<typename std::underlying_type<ModeType>::type>(mode));
}

inline std::ostream& operator<< (std::ostream& os, ModeType mode)
{
  return os << to_string(mode);
}

// Aligned with Transition.srv
enum class TransitionCmd
{
  NO_COMMAND  = 0,
  RESET       = 1,
  START       = 2,
  STOP        = 3,
  HOLD        = 4,
  UNHOLD      = 5,
  SUSPEND     = 6,
  UNSUSPEND   = 7,
  ABORT       = 8,
  CLEAR       = 9
};

inline std::string to_string(const TransitionCmd& command)
{
  switch (command)
  {
    case TransitionCmd::NO_COMMAND: return "NO_COMMAND";
    case TransitionCmd::RESET:      return "RESET";
    case TransitionCmd::START:      return "START";
    case TransitionCmd::STOP:       return "STOP";
    case TransitionCmd::HOLD:       return "HOLD";
    case TransitionCmd::UNHOLD:     return "UNHOLD";
    case TransitionCmd::SUSPEND:    return "SUSPEND";
    case TransitionCmd::UNSUSPEND:  return "UNSUSPEND";
    case TransitionCmd::ABORT:      return "ABORT";
    case TransitionCmd::CLEAR:      return "CLEAR";
  }
  return std::to_string(static_cast<typename std::underlying_type<TransitionCmd>::type>(command));
}

inline std::ostream& operator<< (std::ostream& os, TransitionCmd command)
{
  return os << to_string(command);
}

}  // namespace packml_sm
#endif  // PACKML_SM__COMMON_HPP_
