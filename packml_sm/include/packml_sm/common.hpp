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

#include <map>
#include <mutex>
#include <ostream>
#include <string>
#include <type_traits>
#include <vector>

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

// Map of PackML states to their availability flag, used to configure which
// states are active for a given mode.
using AvailableStates = std::map<State, bool>;

/// Restore the states a mode is not permitted to disable, and report which ones were restored.
///
/// STOPPED, IDLE, EXECUTE and ABORTED are mandatory outright. Three pairs are conditionally
/// mandatory: enabling HOLDING or UNHOLDING makes HELD mandatory, SUSPENDING or UNSUSPENDING
/// makes SUSPENDED mandatory, and COMPLETING makes COMPLETE mandatory -- an acting state whose
/// paired wait state is unavailable has a transition with nowhere to land.
///
/// ABORTING is treated as mandatory here for a different reason: this mask gates which states a
/// command may move the machine INTO, and Abort must be accepted from any state, so masking
/// ABORTING would disable Abort itself.
///
/// A mask is configuration, and repairing it beats refusing to run on it -- a machine that will
/// not start because one flag is wrong helps nobody. Callers are expected to log the returned
/// names so the divergence between the file and the applied mask is visible rather than silent.
inline std::vector<std::string> enforce_mandatory_states(AvailableStates & avail)
{
  std::vector<std::string> restored;
  const auto require = [&avail, &restored](State state) {
      const auto it = avail.find(state);
      if (it == avail.end()) {
        avail[state] = true;              // absent means available; nothing was taken away
        return;
      }
      if (!it->second) {
        it->second = true;
        restored.push_back(to_string(state));
      }
    };
  const auto enabled = [&avail](State state) {
      const auto it = avail.find(state);
      return it == avail.end() || it->second;
    };

  require(State::STOPPED);
  require(State::IDLE);
  require(State::EXECUTE);
  require(State::ABORTED);
  require(State::ABORTING);

  if (enabled(State::HOLDING) || enabled(State::UNHOLDING)) {
    require(State::HELD);
  }
  if (enabled(State::SUSPENDING) || enabled(State::UNSUSPENDING)) {
    require(State::SUSPENDED);
  }
  if (enabled(State::COMPLETING)) {
    require(State::COMPLETE);
  }
  return restored;
}

/// Guards the per-state availability flags that make up a mode's mask
/// (PackmlState::availableInMode()).
///
/// A mode is applied by writing that flag on every state in turn (StatesGenerator::mode_switcher),
/// and it is read when a command is offered a transition (CmdTransition, via
/// target_available_in_current_mode). Those happen on different threads: the mask is written by
/// whoever calls changeMode -- a ROS executor thread, for an operator's mode change -- while
/// commands are evaluated on the state machine's own event-loop thread.
///
/// Two things needed fixing and one lock covers both. The flags are written one at a time, so
/// without holding the lock across the WHOLE loop a command could be evaluated against a
/// half-applied mode -- some states from the new mask, the rest from the old, a combination nobody
/// configured. Hold it for the whole application, not per state. And the write races the read,
/// since the two happen on different threads.
///
/// The flag is a plain bool for a reason worth keeping, and must never become a Qt DYNAMIC
/// PROPERTY: setProperty() on one dispatches a QDynamicPropertyChangeEvent through
/// QCoreApplication::sendEvent SYNCHRONOUSLY on the calling thread -- so the write reached into
/// Qt's notify machinery on an object owned by another thread, which no lock on this side can
/// serialize. See PackmlState::availableInMode().
///
/// A plain lock rather than marshalling the write onto the event-loop thread: that thread already
/// blocks on the ROS executor during a state fan-out (waiting for equipment modules to accept their
/// goals), so making the executor block on it in turn would invert the order -- and the fan-out
/// would then be guaranteed to wait out its full timeout, since the thread that delivers those
/// acceptances would be the one parked. The critical sections here are a bool read and one pass
/// over the state list.
///
/// It also covers the mode VALUE, not just the flags: StatesGenerator::Mode carries the ModeType
/// that selected the mask, mode_switcher() commits both in the same locked pass, and
/// StateMachine::getCurrentMode() / getAvailableStates() read them back under this lock. There is
/// deliberately no second cached copy on StateMachine -- one existed, was assigned after the lock
/// was released, and could therefore be read while it disagreed with the flags it described.
///
/// What this lock does NOT cover: mode_switcher()'s "are we in a mode-switchable state" gate reads
/// the machine's current state under this lock, so two concurrent mode changes cannot interleave
/// their check and their write -- but the state itself is stored by the event-loop thread, which
/// does not take this lock, so a state change can still land immediately after the read. The gate
/// therefore remains advisory against the machine's own thread. Closing that half would mean
/// holding this lock across transition selection itself, which is the inverted design above. The
/// consequence is bounded: the mask is consulted only by CmdTransition, so a late mask can affect
/// the admission of the next operator command and nothing else.
inline std::mutex & mode_mask_mutex()
{
  static std::mutex mask_mutex;
  return mask_mutex;
}

// ModeType is an alias for int, allowing user-defined mode values via
// the packml_sm_generate_modes CMake function.
using ModeType = int;

template<typename T>
std::string to_string(T mode) {
  return std::to_string(static_cast<int>(mode));
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
