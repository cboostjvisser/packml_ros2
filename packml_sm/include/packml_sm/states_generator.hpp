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

namespace packml_sm {
class StatesGenerator {

  // class StateMachine;

public:
  enum class TransitionType { ERROR, STATE_COMPLETED, COMMAND };

  using AvailableStates = std::map<State, bool>;
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
    std::string name;
    // std::map<State, bool> avail_states;
    AvailableStates available_states;

    Mode(std::string name, AvailableStates available_states)
        : name(name), available_states(available_states)
        {}
  };

  Mode currentMode{"", {}};

  inline std::expected<bool, std::string> mode_switcher(std::shared_ptr<StateMachine> sm, Mode mode_to_switch)
  {
    // TODO: Hacky if current mode name is empty; probably uninitialized
    if (switch_states.find(sm->getCurrentState()) != switch_states.end() || currentMode.name.empty())
    {
      for (auto state : mode_to_switch.available_states)
      {
        // TODO: states key enum instead of string?
        auto state1 = states[to_string(state.first)];
        state1->setProperty("Available", state.second);
      }
      currentMode = mode_to_switch;
      std::cout << "Switched mode: " << mode_to_switch.name << std::endl;
      return true;
    }
    else
    {
      std::stringstream msg;
      msg << "Cannot switch mode in state: " << sm->getCurrentState();
      std::cout << msg.str() << std::endl;
      return std::unexpected(msg.str());
    }
    // Cannot reach this
    return false;
  }

  // IDLE  |-CMD Start->  Starting  |-SC->  Execute

  inline void generate_all_packml_states(std::shared_ptr<StateMachine> sm) {
    printf("Forming state machine (states + transitions)\n");
    // Create SuperState
    PackmlSuperState *abortable = PackmlSuperState::Abortable();
    PackmlSuperState *stoppable = PackmlSuperState::Stoppable(abortable);

    // Create Packml states
    ActingState *Aborting = ActingState::Aborting();
    WaitState *Aborted = WaitState::Aborted();
    ActingState *Clearing = ActingState::Clearing(abortable);
    ActingState *Stopping = ActingState::Stopping(abortable);
    WaitState *Stopped = WaitState::Stopped(abortable);
    ActingState *Resetting = ActingState::Resetting(stoppable);
    WaitState *Idle = WaitState::Idle(stoppable);
    ActingState *Starting = ActingState::Starting(stoppable);
    // Dual state; Acting and Waiting state at the same time
    ActingState *Execute = ActingState::Execute(stoppable);
    ActingState *Holding = ActingState::Holding(stoppable);
    WaitState *Held = WaitState::Held(stoppable);
    ActingState *Unholding = ActingState::Unholding(stoppable);
    ActingState *Suspending = ActingState::Suspending(stoppable);
    WaitState *Suspended = WaitState::Suspended(stoppable);
    ActingState *Unsuspending = ActingState::Unsuspending(stoppable);
    ActingState *Completing = ActingState::Completing(stoppable);
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
    auto abortable_aborting_on_error =
        generate_transition(Aborting, TransitionType::ERROR);

    abortable->addTransition(abortable_aborting);
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

    // Set initial states of super states
    abortable->setInitialState(Clearing);
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
            std::cout << "Fell through Transition switch, returning 'No Command' "
                        "transition" << std::endl;
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

  inline void add_state(std::shared_ptr<StateMachine> sm, PackmlState *state) {
    if (states.find(state->name()) == states.end()) {
      states[state->name()] = state;
      std::cout << "Added state: " << state->name() << std::endl;

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
      std::cout << state->name() << ": Already exists" << std::endl;
    }
  }

  // private:
  std::map<std::string, PackmlState *> states;
};

} // namespace packml_sm