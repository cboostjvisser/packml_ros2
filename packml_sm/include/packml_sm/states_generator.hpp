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

#pragma once

#include "packml_sm/common.hpp"
#include "packml_sm/states/acting_state.hpp"
#include "packml_sm/states/state.hpp"
#include "packml_sm/states/toplevel_states.hpp"
#include "packml_sm/states/wait_state.hpp"
#include "packml_sm/transitions/cmd_transition.hpp"
#include "packml_sm/transitions/sc_transition.hpp"
#include "packml_sm/transitions/error_transition.hpp"
#include <map>
#include <qabstracttransition.h>
#include <qchar.h>
#include <vector>

namespace packml_sm
{
    class StatesGenerator {
        public:
        enum class TransitionType
        {
            ERROR,
            STATE_COMPLETED,
            COMMAND
        };



        // IDLE  |-CMD Start->  Starting  |-SC->  Execute

        inline void generate_all_packml_states()
        {
            printf("Forming state machine (states + transitions)\n");
            // Naming <from state>_<to state>

            // Create SuperState
            PackmlSuperState *abortable = PackmlSuperState::Abortable();
            PackmlSuperState *stoppable = PackmlSuperState::Stoppable(abortable);

            // Create Packml states
            ActingState * Aborting = ActingState::Aborting();
            WaitState   * Aborted = WaitState::Aborted();
            ActingState * Clearing = ActingState::Clearing(abortable);
            ActingState * Stopping = ActingState::Stopping(abortable);
            WaitState   * Stopped = WaitState::Stopped(abortable);
            ActingState * Resetting = ActingState::Resetting(stoppable);
            WaitState   * Idle = WaitState::Idle(stoppable);
            ActingState * Starting = ActingState::Starting(stoppable);
            ActingState * Execute = ActingState::Execute(stoppable);
            ActingState * Holding = ActingState::Holding(stoppable);
            WaitState   * Held = WaitState::Held(stoppable);
            ActingState * Unholding = ActingState::Unholding(stoppable);
            ActingState * Suspending = ActingState::Suspending(stoppable);
            WaitState   * Suspended = WaitState::Suspended(stoppable);
            ActingState * Unsuspending = ActingState::Unsuspending(stoppable);
            ActingState * Completing = ActingState::Completing(stoppable);
            WaitState   * Complete = WaitState::Complete(stoppable);

            // TODO: We add abortable state because we need to add it to the state machine. But its not official packml state. See if we can save elsewhere
            add_state(abortable);
            add_state(Aborting);
            add_state(Aborted);
            add_state(Clearing);
            add_state(Stopping);
            add_state(Stopped);
            add_state(Resetting);
            add_state(Idle);
            add_state(Starting);
            // Dual state; Acting and Waiting state at the same time
            add_state(Execute);
            add_state(Holding);
            add_state(Held);
            add_state(Unholding);
            add_state(Suspending);
            add_state(Suspended);
            add_state(Unsuspending);
            add_state(Completing);
            add_state(Complete);

            // Create transitions
            auto abortable_aborting = generate_transition(Aborting, TransitionType::COMMAND);
            auto abortable_aborting_on_error = generate_transition(Aborting, TransitionType::ERROR);

            abortable->addTransition(abortable_aborting);
            abortable->addTransition(abortable_aborting_on_error);

            auto aborting_aborted = generate_transition(Aborted, TransitionType::STATE_COMPLETED);
            Aborting->addTransition(aborting_aborted);

            auto aborted_clearing = generate_transition(Clearing, TransitionType::COMMAND);
            Aborted->addTransition(aborted_clearing);

            auto clearing_stopped = generate_transition(Stopped, TransitionType::STATE_COMPLETED);
            Clearing->addTransition(clearing_stopped);

            auto stoppable_stopping = generate_transition(Stopping, TransitionType::COMMAND);
            stoppable->addTransition(stoppable_stopping);

            auto stopping_stopped = generate_transition(Stopped, TransitionType::STATE_COMPLETED);
            Stopping->addTransition(stopping_stopped);

            auto stopped_resetting = generate_transition(Resetting, TransitionType::COMMAND);
            Stopped->addTransition(stopped_resetting);

            auto resetting_idle = generate_transition(Idle, TransitionType::STATE_COMPLETED);
            Resetting->addTransition(resetting_idle);

            auto idle_starting = generate_transition(Starting, TransitionType::COMMAND);
            Idle->addTransition(idle_starting);

            auto starting_execute = generate_transition(Execute, TransitionType::STATE_COMPLETED);
            Starting->addTransition(starting_execute);

            auto execute_holding = generate_transition(Holding, TransitionType::COMMAND);
            Execute->addTransition(execute_holding);

            auto holding_held = generate_transition(Held, TransitionType::STATE_COMPLETED);
            Holding->addTransition(holding_held);

            auto held_unholding = generate_transition(Unholding, TransitionType::COMMAND);
            Held->addTransition(held_unholding);

            auto unholding_execute = generate_transition(Execute, TransitionType::STATE_COMPLETED);
            Unholding->addTransition(unholding_execute);

            auto execute_suspending = generate_transition(Suspending, TransitionType::COMMAND);
            Execute->addTransition(execute_suspending);

            auto suspending_suspended = generate_transition(Suspended, TransitionType::STATE_COMPLETED);
            Suspending->addTransition(suspending_suspended);

            auto suspended_unsuspending = generate_transition(Unsuspending, TransitionType::COMMAND);
            Suspended->addTransition(suspended_unsuspending);

            auto unsuspending_execute = generate_transition(Execute, TransitionType::STATE_COMPLETED);
            Unsuspending->addTransition(unsuspending_execute);

            auto execute_completing = generate_transition(Completing, TransitionType::STATE_COMPLETED);
            Execute->addTransition(execute_completing);

            auto completing_complete = generate_transition(Complete, TransitionType::STATE_COMPLETED);
            Completing->addTransition(completing_complete);

            auto complete_resetting = generate_transition(Resetting, TransitionType::COMMAND);
            Complete->addTransition(complete_resetting);

            // Set initial states of super states
            abortable->setInitialState(Clearing);
            stoppable->setInitialState(Resetting);

            // Don't forget to set state machine initial state, currently set elsewhere
            // sm_internal_.setInitialState(Aborted);
            printf("State machine formed\n");
        }

        inline QAbstractTransition* generate_transition(PackmlState * transition_to, TransitionType trans_type)
        {
            QAbstractTransition* transition;
            if (trans_type == TransitionType::ERROR)
            {
                transition = new ErrorTransition();
            }
            else if (trans_type == TransitionType::STATE_COMPLETED)
            {
                transition = new StateCompleteTransition();
            }
            else if (trans_type == TransitionType::COMMAND)
            {
                // TODO: pass transition command instead of checking target state
                switch (transition_to->state()) {
                case State::CLEARING:
                {
                    transition = CmdTransition::clear();
                    break;
                }
                case State::STARTING:
                {
                    transition = CmdTransition::start();
                    break;
                }
                case State::STOPPING:
                {
                    transition = CmdTransition::stop();
                    break;
                }
                case State::ABORTING:
                {
                    transition = CmdTransition::abort();
                    break;
                }
                case State::HOLDING:
                {
                    transition = CmdTransition::hold();
                    break;
                }
                case State::UNHOLDING:
                {
                    transition = CmdTransition::unhold();
                    break;
                }
                case State::SUSPENDING:
                {
                    transition = CmdTransition::suspend();
                    break;
                }
                case State::UNSUSPENDING:
                {
                    transition = CmdTransition::unsuspend();
                    break;
                }
                case State::RESETTING:
                {
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
                    std::cout << "Fell through Transition switch, returning 'No Command' transition" << std::endl;
                    transition = new CmdTransition(TransitionCmd::NO_COMMAND, "No_Command");
                }
            }
            else {
                transition = new CmdTransition(TransitionCmd::NO_COMMAND, "No_Command");
                // transition = CmdTransition(TransitionCmd::NO_COMMAND, QString(to_string(TransitionCmd::NO_COMMAND)));
            }

            transition->setTargetState(transition_to);

            return transition;
        }

        inline void add_transition_to_state(PackmlState * state, QAbstractTransition * transition)
        {
            state->addTransition(transition);
        }

        inline void add_state(PackmlState * state) {
            if (states.find(state->name()) == states.end())
            {
                states[state->name()] = state;
                std::cout << "Added state: " << state->name() << std::endl;

                // transition->setTargetState(state);
                // previous_state->addTransition(transition);
            }
            else {
                std::cout << state->name() << ": Already exists" << std::endl;
            }
        }

    // private:
        std::map<std::string, PackmlState*> states;
    };

} // namespace packml_sm