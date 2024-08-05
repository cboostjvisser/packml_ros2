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
        enum class TransitionType
        {
            ERROR,
            STATE_COMPLETED,
            COMMAND
        };

        public:

        // IDLE  |-CMD Start->  Starting  |-SC->  Execute

        inline void generate_all_packml_states()
        {
            printf("Forming state machine (states + transitions)\n");
            // Naming <from state>_<to state>
            PackmlSuperState *abortable = PackmlSuperState::Abortable();
            PackmlSuperState *stoppable = PackmlSuperState::Stoppable(abortable);

            ActingState * Aborting = ActingState::Aborting();

            auto abortable_aborting = generate_transition(Aborting, TransitionType::COMMAND);
            auto abortable_aborting_on_error = generate_transition(Aborting, TransitionType::ERROR);

            abortable->addTransition(abortable_aborting);
            abortable->addTransition(abortable_aborting_on_error);

            this->add_state(Aborting);

            add_state(abortable);


            WaitState * Aborted = WaitState::Aborted();

            auto aborting_aborted = generate_transition(Aborted, TransitionType::STATE_COMPLETED);
            Aborting->addTransition(aborting_aborted);

            this->add_state(Aborted);

            ActingState * Clearing = ActingState::Clearing(abortable);

            auto aborted_clearing = generate_transition(Clearing, TransitionType::COMMAND);
            Aborted->addTransition(aborted_clearing);

            this->add_state(Clearing);

            WaitState * Stopped = WaitState::Stopped(abortable);

            auto clearing_stopped = generate_transition(Stopped, TransitionType::STATE_COMPLETED);
            Clearing->addTransition(clearing_stopped);


            ActingState * Stopping = ActingState::Stopping(abortable);
            auto stoppable_stopping = generate_transition(Stopping, TransitionType::COMMAND);
            stoppable->addTransition(stoppable_stopping);
            this->add_state(Stopping);

            // WaitState * Stopped = WaitState::Stopped(abortable);
            auto stopping_stopped = generate_transition(Stopped, TransitionType::STATE_COMPLETED);
            Stopping->addTransition(stopping_stopped);

            this->add_state(Stopped);

            ActingState * Resetting = ActingState::Resetting(stoppable);
            auto stopped_resetting = generate_transition(Resetting, TransitionType::COMMAND);
            Stopped->addTransition(stopped_resetting);
            this->add_state(Resetting);

            WaitState * Idle = WaitState::Idle(stoppable);
            auto resetting_idle = generate_transition(Idle, TransitionType::STATE_COMPLETED);
            Resetting->addTransition(resetting_idle);
            this->add_state(Idle);

            ActingState * Starting = ActingState::Starting(stoppable);
            auto idle_starting = generate_transition(Starting, TransitionType::COMMAND);
            Idle->addTransition(idle_starting);
            add_state(Starting);

            // Dual state; Acting and Waiting state at the same time
            ActingState * Execute = ActingState::Execute(stoppable);
            auto starting_execute = generate_transition(Execute, TransitionType::STATE_COMPLETED);
            Starting->addTransition(starting_execute);
            add_state(Execute);

            ActingState * Holding = ActingState::Holding(stoppable);
            auto execute_holding = generate_transition(Holding, TransitionType::COMMAND);
            Execute->addTransition(execute_holding);
            add_state(Holding);

            WaitState * Held = WaitState::Held(stoppable);
            auto holding_held = generate_transition(Held, TransitionType::STATE_COMPLETED);
            Holding->addTransition(holding_held);
            add_state(Held);

            ActingState * Unholding = ActingState::Unholding(stoppable);
            auto held_unholding = generate_transition(Unholding, TransitionType::COMMAND);
            Held->addTransition(held_unholding);
            add_state(Unholding);

            auto unholding_execute = generate_transition(Execute, TransitionType::STATE_COMPLETED);
            Unholding->addTransition(unholding_execute);

            ActingState * Suspending = ActingState::Suspending(stoppable);
            auto execute_suspending = generate_transition(Suspending, TransitionType::COMMAND);
            Execute->addTransition(execute_suspending);
            add_state(Suspending);

            WaitState * Suspended = WaitState::Suspended(stoppable);
            auto suspending_suspended = generate_transition(Suspended, TransitionType::STATE_COMPLETED);
            Suspending->addTransition(suspending_suspended);
            add_state(Suspended);

            ActingState * Unsuspending = ActingState::Unsuspending(stoppable);
            auto suspended_unsuspending = generate_transition(Unsuspending, TransitionType::COMMAND);
            Suspended->addTransition(suspended_unsuspending);
            add_state(Unsuspending);

            auto unsuspending_execute = generate_transition(Execute, TransitionType::STATE_COMPLETED);
            Unsuspending->addTransition(unsuspending_execute);

            ActingState * Completing = ActingState::Completing(stoppable);
            auto execute_completing = generate_transition(Completing, TransitionType::STATE_COMPLETED);
            Execute->addTransition(execute_completing);
            add_state(Completing);

            WaitState * Complete = WaitState::Complete(stoppable);
            auto completing_complete = generate_transition(Complete, TransitionType::STATE_COMPLETED);
            Completing->addTransition(completing_complete);
            add_state(Complete);

            auto complete_resetting = generate_transition(Resetting, TransitionType::COMMAND);
            Complete->addTransition(complete_resetting);

            abortable->setInitialState(Clearing);
            stoppable->setInitialState(Resetting);

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