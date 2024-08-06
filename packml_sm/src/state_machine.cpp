// Copyright (c) 2016 Shaun Edwards
// Copyright (c) 2019 Dejanira Araiza Illan, ROS-Industrial Asia Pacific
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

#include "packml_sm/state_machine.hpp"

#include "packml_sm/common.hpp"
#include "packml_sm/states_generator.hpp"
#include "packml_sm/transitions/cmd_transition.hpp"
#include "packml_sm/transitions/sc_transition.hpp"
#include "packml_sm/transitions/error_transition.hpp"

// #include "packml_sm/events.hpp"
#include "packml_sm/states/acting_state.hpp"
#include <iostream>
#include <memory>
#include <string>

namespace packml_sm {

bool StateMachineInterface::start() {
  _start();
  return true;
  // switch (State(getCurrentState())) {
  // case State::IDLE:
  //   _start();
  //   return true;
  // default:
  //   std::cout << "Ignoring START command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::clear() {
  _clear();
  return true;
  // switch (State(getCurrentState())) {
  // case State::ABORTED:
  //   _clear();
  //   return true;
  // default:
  //   std::cout << "Ignoring CLEAR command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::reset() {
  _reset();
  return true;
  // switch (State(getCurrentState())) {
  // case State::COMPLETE:
  // case State::STOPPED:
  //   _reset();
  //   return true;
  // default:
  //   std::cout << "Ignoring RESET command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::hold() {
  _hold();
  return true;
  // switch (State(getCurrentState())) {
  // case State::EXECUTE:
  //   _hold();
  //   return true;
  // default:
  //   std::cout << "Ignoring HOLD command in current state: " << getCurrentState()
  //             << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::unhold() {
  _unhold();
  return true;
  // switch (State(getCurrentState())) {
  // case State::HELD:
  //   _unhold();
  //   return true;
  // default:
  //   std::cout << "Ignoring HELD command in current state: " << getCurrentState()
  //             << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::suspend() {
  _suspend();
  return true;
  // switch (State(getCurrentState())) {
  // case State::EXECUTE:
  //   _suspend();
  //   return true;
  // default:
  //   std::cout << "Ignoring SUSPEND command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::unsuspend() {
  _unsuspend();
  return true;
  // switch (State(getCurrentState())) {
  // case State::SUSPENDED:
  //   _unsuspend();
  //   return true;
  // default:
  //   std::cout << "Ignoring UNSUSPEND command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::stop() {
  _stop();
  return true;
  // switch (State(getCurrentState())) {
  // // case StatesEnum::STOPPABLE:
  // case State::STARTING:
  // case State::IDLE:
  // case State::SUSPENDED:
  // case State::EXECUTE:
  // case State::HOLDING:
  // case State::HELD:
  // case State::SUSPENDING:
  // case State::UNSUSPENDING:
  // case State::UNHOLDING:
  // case State::COMPLETING:
  // case State::COMPLETE:
  //   _stop();
  //   return true;
  // default:
  //   std::cout << "Ignoring STOP command in current state: " << getCurrentState()
  //             << std::endl;
  //   return false;
  // }
}

bool StateMachineInterface::abort() {
  _abort();
  return true;
  // switch (State(getCurrentState())) {
  // // case StatesEnum::ABORTABLE:
  // case State::STOPPED:
  // case State::STARTING:
  // case State::IDLE:
  // case State::SUSPENDED:
  // case State::EXECUTE:
  // case State::HOLDING:
  // case State::HELD:
  // case State::SUSPENDING:
  // case State::UNSUSPENDING:
  // case State::UNHOLDING:
  // case State::COMPLETING:
  // case State::COMPLETE:
  // case State::CLEARING:
  // case State::STOPPING:
  //   _abort();
  //   return true;
  // default:
  //   std::cout << "Ignoring ABORT command in current state: "
  //             << getCurrentState() << std::endl;
  //   return false;
  // }
}

QCoreApplication *a;
void init(int argc, char *argv[]) {
  if (NULL == QCoreApplication::instance()) {
    printf("Starting QCoreApplication\n");
    a = new QCoreApplication(argc, argv);
  }
}

bool StateMachine::activate() {
  printf("Checking if QCore application is running\n");
  if (NULL == QCoreApplication::instance()) {
    printf("QCore application is not running, QCoreApplication must be created "
           "in main");
    printf(" thread for state machine to run\n");
    return false;
  } else {
    printf("Moving state machine to Qcore thread\n");
    sm_internal_.moveToThread(QCoreApplication::instance()->thread());
    this->moveToThread(QCoreApplication::instance()->thread());
    sm_internal_.start();
    printf("State machine thread created and started\n");
    return true;
  }
}

bool StateMachine::deactivate() {
  printf("Deactivating state machine\n");
  sm_internal_.stop();
  return true;
}


std::shared_ptr<StateMachine> StateMachine::singleCyleSM() {
  return std::shared_ptr<StateMachine>(new SingleCycle());
}

std::shared_ptr<StateMachine> StateMachine::continuousCycleSM() {
  return std::shared_ptr<StateMachine>(new ContinuousCycle());
}

/*
 * NOTES:
 * Create factory methods that take std::bind as an argument for
 * a custom call back in the "onExit" method.
 *
 * StateMachine will consist of several public SLOTS for each
 * PackML command.  The implementations will post events to the SM
 * when called.
 *
 * Specializations of StateMachine (like ROS StateMachine) will use
 * state entered events to trigger status publishing via SLOTS
 *
 * Mode handling will be achieved using a hiearchy of state machines
 * that reference/utilize many of the same transitions/states (maybe)
 */

StateMachine::StateMachine() {
  printf("State machine constructor\n");
  // printf("Constructiong super states\n");
  abortable_ = PackmlSuperState::Abortable();
  stoppable_ = PackmlSuperState::Stoppable(abortable_);

  // printf("Constructiong acting/wait states\n");
  held_ = WaitState::Held(stoppable_);
  idle_ = WaitState::Idle(stoppable_);
  complete_ = WaitState::Complete(stoppable_);
  suspended_ = WaitState::Suspended(stoppable_);
  stopped_ = WaitState::Stopped(abortable_);
  aborted_ = WaitState::Aborted();
  unholding_ = ActingState::Unholding(stoppable_, 200);
  holding_ = ActingState::Holding(stoppable_, 200);
  starting_ = ActingState::Starting(stoppable_);
  completing_ = ActingState::Completing(stoppable_);
  resetting_ = ActingState::Resetting(stoppable_);
  unsuspending_ = ActingState::Unsuspending(stoppable_);
  suspending_ = ActingState::Suspending(stoppable_);
  stopping_ = ActingState::Stopping(abortable_);
  clearing_ = ActingState::Clearing(abortable_);
  aborting_ = ActingState::Aborting();
  execute_ = ActingState::Execute(stoppable_);

  connect(abortable_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(stoppable_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(unholding_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(held_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(holding_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(idle_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(starting_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(completing_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(complete_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(resetting_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(unsuspending_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(suspended_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(suspending_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(stopped_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(stopping_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(clearing_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(aborted_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(aborting_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  connect(execute_, SIGNAL(stateEntered(State, QString)), this,
          SLOT(setState(State, QString))); // NOLINT(whitespace/comma)
  printf("Adding states to state machine\n");
  sm_internal_.addState(abortable_);
  sm_internal_.addState(aborted_);
  sm_internal_.addState(aborting_);
}

void StateMachine::setState(State value, QString name) {
  std::string nameUtf = name.toStdString();
  std::cout << "State changed(event) to: " << nameUtf << "(" << value << ")"
            << std::endl;
  state_value_ = value;
  state_name_ = name;
  emit stateChanged(value, name);
}

bool StateMachine::setExecute(std::function<int()> execute_method) {
  printf("Initializing state machine with EXECUTE function pointer\n");
  return execute_->setOperationMethod(execute_method);
}

bool StateMachine::setResetting(std::function<int()> resetting_method) {
  printf("Initializing state machine with RESETTING function pointer\n");
  return resetting_->setOperationMethod(resetting_method);
}

void StateMachine::_start() { sm_internal_.postEvent(CmdEvent::start()); }
void StateMachine::_clear() { sm_internal_.postEvent(CmdEvent::clear()); }
void StateMachine::_reset() { sm_internal_.postEvent(CmdEvent::reset()); }
void StateMachine::_hold() { sm_internal_.postEvent(CmdEvent::hold()); }
void StateMachine::_unhold() { sm_internal_.postEvent(CmdEvent::unhold()); }
void StateMachine::_suspend() { sm_internal_.postEvent(CmdEvent::suspend()); }
void StateMachine::_unsuspend() { sm_internal_.postEvent(CmdEvent::unsuspend()); }
void StateMachine::_stop() { sm_internal_.postEvent(CmdEvent::stop()); }
void StateMachine::_abort() { sm_internal_.postEvent(CmdEvent::abort()); }

ContinuousCycle::ContinuousCycle() {
  printf("Forming CONTINUOUS CYCLE state machine (states + transitions)\n");
  // Naming <from state>_<to state>
  CmdTransition::abort(*abortable_, *aborting_);
  ErrorTransition *abortable_aborting_on_error =
      new ErrorTransition(*abortable_, *aborting_);

  StateCompleteTransition *aborting_aborted =
      new StateCompleteTransition(*aborting_, *aborted_);

  CmdTransition::clear(*aborted_, *clearing_);
  StateCompleteTransition *clearing_stopped_ =
      new StateCompleteTransition(*clearing_, *stopped_);

  CmdTransition::stop(*stoppable_, *stopping_);
  StateCompleteTransition *stopping_stopped =
      new StateCompleteTransition(*stopping_, *stopped_);

  CmdTransition::reset(*stopped_, *resetting_);
  StateCompleteTransition *unholding_execute_ =
      new StateCompleteTransition(*unholding_, *execute_);

  CmdTransition::unhold(*held_, *unholding_);
  StateCompleteTransition *holding_held_ =
      new StateCompleteTransition(*holding_, *held_);

  CmdTransition::start(*idle_, *starting_);
  StateCompleteTransition *starting_execute_ =
      new StateCompleteTransition(*starting_, *execute_);

  CmdTransition::hold(*execute_, *holding_);
  StateCompleteTransition *execute_execute_ =
      new StateCompleteTransition(*execute_, *execute_);

  StateCompleteTransition *completing_complete =
      new StateCompleteTransition(*completing_, *complete_);

  CmdTransition::reset(*complete_, *resetting_);
  StateCompleteTransition *resetting_idle_ =
      new StateCompleteTransition(*resetting_, *idle_);

  CmdTransition::suspend(*execute_, *suspending_);
  StateCompleteTransition *suspending_suspended_ =
      new StateCompleteTransition(*suspending_, *suspended_);

  CmdTransition::unsuspend(*suspended_, *unsuspending_);
  StateCompleteTransition *unsuspending_execute_ =
      new StateCompleteTransition(*unsuspending_, *execute_);

  abortable_->setInitialState(clearing_);
  stoppable_->setInitialState(resetting_);
  sm_internal_.setInitialState(aborted_);
  printf("State machine formed\n");
}

SingleCycle::SingleCycle() {
  printf("Forming SINGLE CYCLE state machine (states + transitions)\n");
  // Naming <from state>_<to state>
  // auto aborttrans = CmdTransition::abort(*abortable_, *aborting_);
  // ErrorTransition *abortable_aborting_on_error =
  //     new ErrorTransition(*abortable_, *aborting_);

  // StateCompleteTransition *aborting_aborted =
  //     new StateCompleteTransition(*aborting_, *aborted_);

  // CmdTransition::clear(*aborted_, *clearing_);
  // StateCompleteTransition *clearing_stopped_ =
  //     new StateCompleteTransition(*clearing_, *stopped_);

  // CmdTransition::stop(*stoppable_, *stopping_);
  // StateCompleteTransition *stopping_stopped =
  //     new StateCompleteTransition(*stopping_, *stopped_);

  // CmdTransition::reset(*stopped_, *resetting_);
  // StateCompleteTransition *unholding_execute_ =
  //     new StateCompleteTransition(*unholding_, *execute_);

  // CmdTransition::unhold(*held_, *unholding_);
  // StateCompleteTransition *holding_held_ =
  //     new StateCompleteTransition(*holding_, *held_);

  // CmdTransition::start(*idle_, *starting_);
  // StateCompleteTransition *starting_execute_ =
  //     new StateCompleteTransition(*starting_, *execute_);

  // CmdTransition::hold(*execute_, *holding_);
  // StateCompleteTransition *execute_completing_ =
  //     new StateCompleteTransition(*execute_, *completing_);

  // StateCompleteTransition *completing_complete =
  //     new StateCompleteTransition(*completing_, *complete_);

  // CmdTransition::reset(*complete_, *resetting_);
  // StateCompleteTransition *resetting_idle_ =
  //     new StateCompleteTransition(*resetting_, *idle_);

  // CmdTransition::suspend(*execute_, *suspending_);
  // StateCompleteTransition *suspending_suspended_ =
  //     new StateCompleteTransition(*suspending_, *suspended_);

  // CmdTransition::unsuspend(*suspended_, *unsuspending_);
  // StateCompleteTransition *unsuspending_execute_ =
  //     new StateCompleteTransition(*unsuspending_, *execute_);

  // abortable_->setInitialState(clearing_);
  // stoppable_->setInitialState(resetting_);

  gen.generate_all_packml_states();
  // Add parent states to state machine
  sm_internal_.addState(gen.states["Abortable"]);
  sm_internal_.addState(gen.states[to_string(State::ABORTED)]);
  sm_internal_.addState(gen.states[to_string(State::ABORTING)]);

  sm_internal_.setInitialState(gen.states[to_string(State::ABORTED)]);
  printf("End of single cycle\n");

  // Test to see if we can adjust the state machines transitions
  auto list = gen.states[to_string(State::EXECUTE)]->transitions();
  for (const auto& item : list)
  {
      if (item->targetState() == gen.states[to_string(State::COMPLETING)])
      {
        std::cout << "Foind transition!" << std::endl;
        gen.states[to_string(State::EXECUTE)]->removeTransition(item);
        std::cout << "Removed transition!" << std::endl;
        auto trans = gen.generate_transition(gen.states[to_string(State::EXECUTE)], StatesGenerator::TransitionType::STATE_COMPLETED);
        gen.states[to_string(State::EXECUTE)]->addTransition(trans);
        std::cout << "Added transition to self!" << std::endl;
      }
  }

}

} // namespace packml_sm
