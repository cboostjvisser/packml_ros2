// Copyright (c) 2016 Shaun Edwards
// Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific (ROS 2
// compatibility)
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

#pragma once

#include "packml_sm/common.hpp"
#include "packml_sm/logging.hpp"
#include "packml_sm/state_machine.hpp"
#include "packml_sm/states/acting_state.hpp"
#include "packml_sm/states/state.hpp"
#include "packml_sm/states/toplevel_states.hpp"
#include "packml_sm/states/wait_state.hpp"
#include "packml_sm/transitions/cmd_transition.hpp"
#include "packml_sm/transitions/error_transition.hpp"
#include "packml_sm/transitions/sc_transition.hpp"
#include <expected>
#include <map>
#include <qabstracttransition.h>
#include <qchar.h>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace packml_sm {
class StatesGenerator {

  // class StateMachine;

public:
  enum class TransitionType { ERROR, STATE_COMPLETED, COMMAND };

  using AvailableStates = packml_sm::AvailableStates;
  using ModeSwitchStates = std::set<State>;

  AvailableStates Available = {
    {State::ABORTING, true},
    {State::ABORTED, true},
    {State::CLEARING, true},
    {State::STOPPING, true},
    {State::STOPPED, true},
    {State::RESETTING, true},
    {State::IDLE, true},
    {State::STARTING, true},
    {State::EXECUTE, true},
    {State::HOLDING, true},
    {State::HELD, true},
    {State::UNHOLDING, true},
    {State::SUSPENDING, true},
    {State::SUSPENDED, true},
    {State::UNSUSPENDING, true},
    {State::COMPLETING, true},
    {State::COMPLETE, true},
  };

  ModeSwitchStates switch_states = {
    State::IDLE
  };



  // struct AvailableStates {
  //   bool Aborting = true;
  //   bool Aborted = true;
  //   bool Clearing = true;
  //   bool Stopped = true;
  //   bool Resetting = true;
  //   bool Idle = true;
  //   bool Starting = true;
  //   bool Execute = true;
  //   bool Holding = true;
  //   bool Held = true;
  //   bool Unholding = true;
  //   bool Suspending = true;
  //   bool Suspended = true;
  //   bool Unsuspending = true;
  //   bool Completing = true;
  //   bool Complete = true;
  // };

  struct Mode {
    /// The value an operator asked for, carried alongside the mask so the applied mode and the
    /// mask that defines it are one object written under one lock. StateMachine::getCurrentMode()
    /// reads it back from here rather than keeping a second copy of its own.
    ModeType value;
    std::string name;
    // std::map<State, bool> avail_states;
    AvailableStates available_states;

    Mode(ModeType value, std::string name, AvailableStates available_states)
        : value(value), name(name), available_states(available_states)
        {}
  };

  Mode currentMode{ModeType{}, "", {}};

  inline std::expected<bool, std::string> mode_switcher(std::shared_ptr<StateMachine> sm, Mode mode_to_switch)
  {
    // Set when the switch is refused, so the logging happens off the lock. Empty means applied.
    std::string refusal;
    {
      // One lock for the whole pass: see mode_mask_mutex(). Per-state locking would still let a
      // command see a half-applied mode. The switchable-state gate is read inside it too, so two
      // concurrent mode changes cannot interleave their check and their write -- what remains
      // outside its reach is the machine's own thread, which can leave the switchable state
      // immediately after this read.
      std::lock_guard<std::mutex> lk(mode_mask_mutex());

      // TODO: Hacky if current mode name is empty; probably uninitialized
      if (switch_states.find(sm->getCurrentState()) == switch_states.end() && !currentMode.name.empty())
      {
        std::stringstream msg;
        msg << "Cannot switch mode in state: " << sm->getCurrentState();
        refusal = msg.str();
      }
      else
      {
        // Resolve every state the mask names BEFORE writing any of them. `states` maps to raw
        // pointers, so operator[] on a key this machine does not have would insert a null and the
        // write would dereference it; and a mask refused halfway through would leave a mode nobody
        // configured in place. Both are avoided by refusing the whole switch.
        std::vector<std::pair<PackmlState *, bool>> resolved;
        resolved.reserve(mode_to_switch.available_states.size());
        for (const auto & [state, available] : mode_to_switch.available_states)
        {
          // TODO: states key enum instead of string?
          const auto it = states.find(to_string(state));
          if (it == states.end() || nullptr == it->second)
          {
            std::stringstream msg;
            msg << "Mode '" << mode_to_switch.name
                << "' names a state this machine does not have: " << state;
            refusal = msg.str();
            break;
          }
          resolved.emplace_back(it->second, available);
        }

        if (refusal.empty())
        {
          for (const auto & [state, available] : resolved)
          {
            state->setAvailableInMode(available);
          }
          // Committed under the same lock as the flags it describes, so the two cannot disagree.
          currentMode = mode_to_switch;
        }
      }
    }

    if (!refusal.empty())
    {
      PACKML_WARN_STREAM("packml_sm", refusal);
      return std::unexpected(refusal);
    }
    PACKML_INFO_STREAM("packml_sm", "Switched mode: " << mode_to_switch.name);
    return true;
  }

  // IDLE  |-CMD Start->  Starting  |-SC->  Execute

  inline void generate_all_packml_states(std::shared_ptr<StateMachine> sm, int delay_ms = 200) {
    printf("Forming state machine (states + transitions)\n");
    // Create SuperState
    PackmlSuperState *abortable = PackmlSuperState::Abortable();
    PackmlSuperState *stoppable = PackmlSuperState::Stoppable(abortable);

    // Create Packml states
    ActingState *Aborting = ActingState::Aborting(delay_ms);
    WaitState *Aborted = WaitState::Aborted();
    ActingState *Clearing = ActingState::Clearing(abortable, delay_ms);
    ActingState *Stopping = ActingState::Stopping(abortable, delay_ms);
    WaitState *Stopped = WaitState::Stopped(abortable);
    ActingState *Resetting = ActingState::Resetting(stoppable, delay_ms);
    WaitState *Idle = WaitState::Idle(stoppable);
    ActingState *Starting = ActingState::Starting(stoppable, delay_ms);
    // Dual state; Acting and Waiting state at the same time
    ActingState *Execute = ActingState::Execute(stoppable, delay_ms);
    ActingState *Holding = ActingState::Holding(stoppable, delay_ms);
    WaitState *Held = WaitState::Held(stoppable);
    ActingState *Unholding = ActingState::Unholding(stoppable, delay_ms);
    ActingState *Suspending = ActingState::Suspending(stoppable, delay_ms);
    WaitState *Suspended = WaitState::Suspended(stoppable);
    ActingState *Unsuspending = ActingState::Unsuspending(stoppable, delay_ms);
    ActingState *Completing = ActingState::Completing(stoppable, delay_ms);
    WaitState *Complete = WaitState::Complete(stoppable);

    // TODO: We add abortable state because we need to add it to the state
    // machine. But its not official packml state. See if we can save elsewhere
    add_state(sm, abortable);

    add_state(sm, Aborting);
    add_state(sm, Aborted);
    add_state(sm, Clearing);
    add_state(sm, Stopping);
    add_state(sm, Stopped);
    add_state(sm, Resetting);
    add_state(sm, Idle);
    add_state(sm, Starting);
    add_state(sm, Execute);
    add_state(sm, Holding);
    add_state(sm, Held);
    add_state(sm, Unholding);
    add_state(sm, Suspending);
    add_state(sm, Suspended);
    add_state(sm, Unsuspending);
    add_state(sm, Completing);
    add_state(sm, Complete);

    // Create transitions
    // Naming <from state>_<to state>
    auto abortable_aborting =
        generate_transition(Aborting, TransitionType::COMMAND);
    abortable->addTransition(abortable_aborting);

    // Kept deliberately, even though every acting state below owns its own ERROR edge to the same
    // target and preempts this one. On Qt 5.15, when a state and one of its ancestors
    // both have a transition matching the event, the descendant's wins regardless of the order
    // they were added, and the ancestor's eventTest is never consulted. So this edge costs nothing
    // where it is redundant, and it is NOT redundant for the five states inside `abortable` that
    // are WaitStates rather than ActingStates (Stopped, Idle, Held, Suspended, Complete): those
    // get no edge of their own from the loop below, and an error event processed while the machine
    // sits in one of them would otherwise match nothing at all.
    auto abortable_aborting_on_error =
        generate_transition(Aborting, TransitionType::ERROR);
    abortable->addTransition(abortable_aborting_on_error);

    auto aborting_aborted =
        generate_transition(Aborted, TransitionType::STATE_COMPLETED);
    Aborting->addTransition(aborting_aborted);

    auto aborted_clearing =
        generate_transition(Clearing, TransitionType::COMMAND);
    Aborted->addTransition(aborted_clearing);

    auto clearing_stopped =
        generate_transition(Stopped, TransitionType::STATE_COMPLETED);
    Clearing->addTransition(clearing_stopped);

    auto stoppable_stopping =
        generate_transition(Stopping, TransitionType::COMMAND);
    stoppable->addTransition(stoppable_stopping);

    auto stopping_stopped =
        generate_transition(Stopped, TransitionType::STATE_COMPLETED);
    Stopping->addTransition(stopping_stopped);

    auto stopped_resetting =
        generate_transition(Resetting, TransitionType::COMMAND);
    Stopped->addTransition(stopped_resetting);

    auto resetting_idle =
        generate_transition(Idle, TransitionType::STATE_COMPLETED);
    Resetting->addTransition(resetting_idle);

    auto idle_starting = generate_transition(Starting, TransitionType::COMMAND);
    Idle->addTransition(idle_starting);

    auto starting_execute =
        generate_transition(Execute, TransitionType::STATE_COMPLETED);
    Starting->addTransition(starting_execute);

    auto execute_holding =
        generate_transition(Holding, TransitionType::COMMAND);
    Execute->addTransition(execute_holding);

    auto holding_held =
        generate_transition(Held, TransitionType::STATE_COMPLETED);
    Holding->addTransition(holding_held);

    auto held_unholding =
        generate_transition(Unholding, TransitionType::COMMAND);
    Held->addTransition(held_unholding);

    auto unholding_execute =
        generate_transition(Execute, TransitionType::STATE_COMPLETED);
    Unholding->addTransition(unholding_execute);

    auto execute_suspending =
        generate_transition(Suspending, TransitionType::COMMAND);
    Execute->addTransition(execute_suspending);

    auto suspending_suspended =
        generate_transition(Suspended, TransitionType::STATE_COMPLETED);
    Suspending->addTransition(suspending_suspended);

    auto suspended_unsuspending =
        generate_transition(Unsuspending, TransitionType::COMMAND);
    Suspended->addTransition(suspended_unsuspending);

    auto unsuspending_execute =
        generate_transition(Execute, TransitionType::STATE_COMPLETED);
    Unsuspending->addTransition(unsuspending_execute);

    auto execute_completing =
        generate_transition(Completing, TransitionType::STATE_COMPLETED);
    Execute->addTransition(execute_completing);

    auto completing_complete =
        generate_transition(Complete, TransitionType::STATE_COMPLETED);
    Completing->addTransition(completing_complete);

    auto complete_resetting =
        generate_transition(Resetting, TransitionType::COMMAND);
    Complete->addTransition(complete_resetting);

    // Every acting state declares where a failure of its own bound operation lands, and OWNS the
    // ERROR transition that takes it there. Derived from the set of acting states rather than
    // hand-listed, so an acting state added to this graph later cannot be left without an error
    // escape.
    //
    // Until now the only error escape was the shared one on `abortable` above. Inheriting the
    // escape from a superstate looks equivalent and is not: it only serves descendants, it silently
    // serves none when the state sits outside that superstate, and nothing anywhere warns. Aborting
    // is exactly that case -- it is constructed parentless (see above: every other state passes a
    // superstate, Aborting and Aborted do not), so it inherited nothing, and abortable's edge
    // targeted Aborting anyway, so inheriting it would have self-looped rather than escaped. A
    // non-zero return from Aborting's own operation therefore posted an ErrorEvent that matched
    // nothing, was discarded by Qt, and left the machine in ABORTING with no reachable exit: its
    // only other out-edge is its own STATE_COMPLETED, which that same failed operation will never
    // post. ABORTING is a live failure source in production, not a theoretical one: it is in
    // packml_ros's kCoordinatedStates list, whose bound operation returns non-zero on a fan-out
    // timeout.
    //
    // Declaration and wiring are two adjacent statements over one variable here so they cannot
    // drift apart. errorEscapeReport() checks the rest: nothing declares itself as its own failure
    // target, and following the targets always terminates.
    //
    // ABORTED is the safe terminal for a failure of ABORTING itself -- the error handler failing
    // must still land somewhere, and CLEAR recovers from there as normal. Every other acting state
    // escalates to ABORTING, where PackML funnels all faults.
    for (auto & entry : states) {
      auto * acting = dynamic_cast<ActingState *>(entry.second);
      if (nullptr == acting) {
        continue;
      }
      PackmlState * target = (State::ABORTING == acting->state())
        ? static_cast<PackmlState *>(Aborted)
        : static_cast<PackmlState *>(Aborting);
      acting->declareFailureTarget(target);
      acting->addTransition(generate_transition(target, TransitionType::ERROR));
    }

    // Set initial states of super states
    // PackML mandates power-on into STOPPED.  abortable's initial substate
    // is Stopped so the SM boots there.  Explicit transitions (e.g.
    // ABORTED→Clearing) still target their declared substate and override
    // the initialState setting per Qt semantics.
    abortable->setInitialState(Stopped);
    stoppable->setInitialState(Resetting);

    // Don't forget to set state machine initial state, currently set elsewhere
    // sm_internal_.setInitialState(Aborted);
    printf("State machine formed\n");
  }

  inline QAbstractTransition *generate_transition(PackmlState *transition_to,
                                                  TransitionType trans_type) {
    QAbstractTransition *transition;
    if (trans_type == TransitionType::ERROR) {
      transition = new ErrorTransition(); // NOLINT, this is how qt works
    } else if (trans_type == TransitionType::STATE_COMPLETED) {
      transition = new StateCompleteTransition(); // NOLINT, this is how qt works
    } else if (trans_type == TransitionType::COMMAND) {
      // TODO: pass transition command instead of checking target state
      switch (transition_to->state()) {
        case State::CLEARING: {
          transition = CmdTransition::clear();
          break;
        }
        case State::STARTING: {
          transition = CmdTransition::start();
          break;
        }
        case State::STOPPING: {
          transition = CmdTransition::stop();
          break;
        }
        case State::ABORTING: {
          transition = CmdTransition::abort();
          break;
        }
        case State::HOLDING: {
          transition = CmdTransition::hold();
          break;
        }
        case State::UNHOLDING: {
          transition = CmdTransition::unhold();
          break;
        }
        case State::SUSPENDING: {
          transition = CmdTransition::suspend();
          break;
        }
        case State::UNSUSPENDING: {
          transition = CmdTransition::unsuspend();
          break;
        }
        case State::RESETTING: {
          transition = CmdTransition::reset();
          break;
        }
        case State::UNDEFINED:
        case State::STOPPED:
        case State::IDLE:
        case State::SUSPENDED:
        case State::EXECUTE:
        case State::ABORTED:
        case State::HELD:
        case State::COMPLETING:
        case State::COMPLETE:
        default: {
            PACKML_WARN("packml_sm", "Fell through Transition switch, returning 'No Command' transition");
            transition = new CmdTransition(TransitionCmd::NO_COMMAND, "No_Command"); // NOLINT, this is how qt works
        }
      }
    } else {
      transition = new CmdTransition(TransitionCmd::NO_COMMAND, "No_Command"); // NOLINT, this is how qt works
      // transition = CmdTransition(TransitionCmd::NO_COMMAND,
      // QString(to_string(TransitionCmd::NO_COMMAND)));
    }

    transition->setTargetState(transition_to);

    return transition;
  }

  inline void add_transition_to_state(PackmlState *state,
                                      QAbstractTransition *transition) {
    state->addTransition(transition);
  }

  // Everything wrong with the failure-target wiring, one string per fault. Must be empty for the
  // graph to be safe to run: an acting state whose bound operation can fail but whose failure has
  // no reachable destination leaves the machine wedged in that state, because Qt silently discards
  // an event that matches no transition -- no status change, no alarm, no timeout, no log.
  // Reported rather than asserted so the caller can name every offender and then refuse to
  // activate (see StateMachine::activate()).
  //
  // Three faults are checked. The first is the one that shipped. The second and third are the only
  // ways the wiring loop in generate_all_packml_states can still be wrong now that declaration and
  // edge are written as one pair of adjacent statements:
  //   1. an acting state that owns no ERROR transition to its declared failure target;
  //   2. an acting state that declares ITSELF as its failure target, which turns a failure into a
  //      self-loop that re-runs the same failing operation forever;
  //   3. a failure-target chain that does not terminate. Today's chain is ten states -> ABORTING ->
  //      ABORTED, and it ends only because ABORTED is a WaitState with no operation that could
  //      fail. Declare ABORTING's target as CLEARING instead and CLEARING -> ABORTING -> CLEARING
  //      is an unbounded abort loop that no single-state check would notice.
  inline std::vector<std::string> error_escape_report() const {
    std::vector<std::string> faults;
    for (const auto & entry : states) {
      auto * acting = dynamic_cast<ActingState *>(entry.second);
      if (nullptr == acting) {
        continue;
      }
      PackmlState * target = acting->failureTarget();

      bool owns_escape = false;
      if (nullptr != target) {
        for (auto * transition : acting->transitions()) {
          if (nullptr != dynamic_cast<ErrorTransition *>(transition) &&
              transition->targetState() == target) {
            owns_escape = true;
            break;
          }
        }
      }
      if (!owns_escape) {
        faults.push_back(acting->name() + ": owns no ERROR transition to its failure target");
        continue;
      }

      if (target == acting) {
        faults.push_back(acting->name() + ": declares itself as its own failure target");
        continue;
      }

      // Walk the chain. Bounded by the number of states, so a cycle cannot spin here.
      std::set<const PackmlState *> visited{acting};
      const PackmlState * hop = target;
      while (true) {
        const auto * hop_acting = dynamic_cast<const ActingState *>(hop);
        if (nullptr == hop_acting) {
          break;  // terminates on a state whose operation cannot fail
        }
        if (!visited.insert(hop).second) {
          faults.push_back(acting->name() + ": failure targets form a cycle via " + hop->name());
          break;
        }
        hop = hop_acting->failureTarget();
        if (nullptr == hop) {
          break;  // already reported against that state by check 1
        }
      }
    }
    return faults;
  }

  inline void add_state(std::shared_ptr<StateMachine> sm, PackmlState *state) {
    if (states.find(state->name()) == states.end()) {
      states[state->name()] = state;
      PACKML_DEBUG_STREAM("packml_sm", "Added state: " << state->name());

      // Every state starts available, so a freshly created machine is usable without an explicit
      // changeMode(); a mode mask narrows it. The flag is a defaulted member (see
      // PackmlState::availableInMode()), and this write is the explicit statement of that intent.
      {
        std::lock_guard<std::mutex> lk(mode_mask_mutex());
        state->setAvailableInMode(true);
      }

      // auto function = std::bind(StateMachine::setState )
      // Hacky way to filter out superstates. This way we do not get events from super states.
      if (state->state() != State::UNDEFINED) {
        // Connect State Entered Event to Set State function
        StateMachine::connect(state, &PackmlState::stateEntered, sm.get(),
          &StateMachine::setState); // NOLINT(whitespace/comma)
      }
      // transition->setTargetState(state);
      // previous_state->addTransition(transition);
    } else {
      PACKML_WARN_STREAM("packml_sm", state->name() << ": Already exists");
    }
  }

  // private:
  std::map<std::string, PackmlState *> states;
};

} // namespace packml_sm