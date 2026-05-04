// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---
// State-transition coverage tests.  Walks the full PackML state diagram
// exercising every legal transition once.  The "command rejection" matrix lives in
// test_commands.cpp; this file focuses on the *positive* path.
// ---

#include <chrono>
#include <gtest/gtest.h>

#include "packml_sm/state_machine.hpp"
#include "test_helpers.hpp"

using packml_sm::State;
using packml_sm_test::wait_for_state;
using packml_sm_test::execute_success_long;
using packml_sm_test::drive_to_idle;

namespace
{
// All tests in this file use a continuous-cycle SM so EXECUTE does not
// auto-complete -- this lets us exercise HOLD/SUSPEND/STOP from EXECUTE.
auto make_sm()
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  return sm;
}
}  // namespace

// ABORTED --CLEAR--> CLEARING --(SC)--> STOPPED
TEST(Transitions, AbortedClearsToStopped)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  ASSERT_TRUE(packml_sm_test::enable_all_transitions(*sm));
  // First navigate to ABORTED (SM boots in STOPPED now).
  ASSERT_TRUE(sm->abort());
  ASSERT_TRUE(wait_for_state(*sm, State::ABORTED));
  EXPECT_TRUE(sm->clear());
  // CLEARING is acting; we observe its terminal STOPPED.
  EXPECT_TRUE(wait_for_state(*sm, State::STOPPED));
  sm->deactivate();
}

// STOPPED --RESET--> RESETTING --(SC)--> IDLE
TEST(Transitions, StoppedResetsToIdle)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  ASSERT_TRUE(packml_sm_test::enable_all_transitions(*sm));
  EXPECT_TRUE(sm->reset());
  EXPECT_TRUE(wait_for_state(*sm, State::IDLE));
  sm->deactivate();
}

// IDLE --START--> STARTING --(SC)--> EXECUTE
TEST(Transitions, IdleStartsToExecute)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  EXPECT_TRUE(sm->start());
  EXPECT_TRUE(wait_for_state(*sm, State::EXECUTE));
  sm->deactivate();
}

// EXECUTE --HOLD--> HOLDING --(SC)--> HELD --UNHOLD--> UNHOLDING --(SC)--> EXECUTE
TEST(Transitions, ExecuteHoldUnholdRoundTrip)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));

  EXPECT_TRUE(sm->hold());
  EXPECT_TRUE(wait_for_state(*sm, State::HELD, std::chrono::seconds(5)))
    << "HOLDING is acting (default 200ms delay) and must auto-progress to HELD.";

  EXPECT_TRUE(sm->unhold());
  EXPECT_TRUE(wait_for_state(*sm, State::EXECUTE, std::chrono::seconds(5)));
  sm->deactivate();
}

// EXECUTE --SUSPEND--> SUSPENDING --(SC)--> SUSPENDED --UNSUSPEND-->
// UNSUSPENDING --(SC)--> EXECUTE
TEST(Transitions, ExecuteSuspendUnsuspendRoundTrip)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));

  EXPECT_TRUE(sm->suspend());
  EXPECT_TRUE(wait_for_state(*sm, State::SUSPENDED, std::chrono::seconds(5)));

  EXPECT_TRUE(sm->unsuspend());
  EXPECT_TRUE(wait_for_state(*sm, State::EXECUTE, std::chrono::seconds(5)));
  sm->deactivate();
}

// PackML stoppable group: STARTING/IDLE/SUSPENDED/EXECUTE/HOLDING/HELD/
// SUSPENDING/UNSUSPENDING/UNHOLDING/COMPLETING/COMPLETE all accept STOP.
// Exercised here from EXECUTE.
TEST(Transitions, ExecuteStopsToStopped)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  EXPECT_TRUE(sm->stop());
  EXPECT_TRUE(wait_for_state(*sm, State::STOPPED));
  sm->deactivate();
}

// PackML stoppable group: STOP accepted from IDLE.
TEST(Transitions, IdleStopsToStopped)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  EXPECT_TRUE(sm->stop());
  EXPECT_TRUE(wait_for_state(*sm, State::STOPPED));
  sm->deactivate();
}

// PackML stoppable group: STOP accepted from HELD.
TEST(Transitions, HeldStopsToStopped)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  ASSERT_TRUE(sm->hold());
  ASSERT_TRUE(wait_for_state(*sm, State::HELD, std::chrono::seconds(5)));
  EXPECT_TRUE(sm->stop());
  EXPECT_TRUE(wait_for_state(*sm, State::STOPPED));
  sm->deactivate();
}

// PackML stoppable group: STOP accepted from SUSPENDED.
TEST(Transitions, SuspendedStopsToStopped)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  ASSERT_TRUE(sm->suspend());
  ASSERT_TRUE(wait_for_state(*sm, State::SUSPENDED, std::chrono::seconds(5)));
  EXPECT_TRUE(sm->stop());
  EXPECT_TRUE(wait_for_state(*sm, State::STOPPED));
  sm->deactivate();
}

// PackML abortable group accepts ABORT from anywhere except ABORTED/ABORTING.
// Exercised from EXECUTE here; from STOPPED in a separate test.
TEST(Transitions, ExecuteAbortsToAborted)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  EXPECT_TRUE(sm->abort());
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED));
  sm->deactivate();
}

TEST(Transitions, StoppedAbortsToAborted)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  ASSERT_TRUE(packml_sm_test::enable_all_transitions(*sm));
  EXPECT_TRUE(sm->abort());
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED));
  sm->deactivate();
}

// Single-cycle: EXECUTE --(SC)--> COMPLETING --(SC)--> COMPLETE
TEST(Transitions, SingleCycleExecuteCompletesToComplete)
{
  auto sm = packml_sm::StateMachine::singleCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(packml_sm_test::execute_success_long));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  EXPECT_TRUE(wait_for_state(*sm, State::COMPLETE, std::chrono::seconds(5)));
  sm->deactivate();
}

// COMPLETE --RESET--> RESETTING --(SC)--> IDLE
TEST(Transitions, CompleteResetsToIdle)
{
  auto sm = packml_sm::StateMachine::singleCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(packml_sm_test::execute_success_long));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::COMPLETE, std::chrono::seconds(5)));
  EXPECT_TRUE(sm->reset());
  EXPECT_TRUE(wait_for_state(*sm, State::IDLE));
  sm->deactivate();
}

// COMPLETE --STOP--> STOPPING --(SC)--> STOPPED
TEST(Transitions, CompleteStopsToStopped)
{
  auto sm = packml_sm::StateMachine::singleCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(packml_sm_test::execute_success_long));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::COMPLETE, std::chrono::seconds(5)));
  EXPECT_TRUE(sm->stop());
  EXPECT_TRUE(wait_for_state(*sm, State::STOPPED));
  sm->deactivate();
}

// changeState() entry-point: same outcome as direct command methods.
TEST(Transitions, ChangeStateRoutesCommandsCorrectly)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  ASSERT_TRUE(packml_sm_test::enable_all_transitions(*sm));

  auto rc = sm->changeState(packml_sm::TransitionCmd::RESET);
  EXPECT_TRUE(rc.has_value()) << (rc.has_value() ? "" : rc.error());
  EXPECT_TRUE(wait_for_state(*sm, State::IDLE));

  // NO_COMMAND is invalid for changeState.
  auto bad = sm->changeState(packml_sm::TransitionCmd::NO_COMMAND);
  EXPECT_FALSE(bad.has_value())
    << "changeState(NO_COMMAND) must return std::unexpected.";
  sm->deactivate();
}

// PackML PackTags require per-state cumulative time.  Verify that accessing
// a state after it has been entered and exited produces a non-zero duration.
TEST(Transitions, CumulativeTimeIsTracked)
{
  auto sm = make_sm();
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  // RESETTING has already been entered and exited to reach IDLE; its
  // cumulative time must be > 0.
  double resetting_time = sm->getStateCumulativeTime(State::RESETTING);
  EXPECT_GT(resetting_time, 0.0)
    << "Cumulative time for RESETTING should be > 0 after passing through it.";
  // EXECUTE hasn't been entered yet — should be 0.
  EXPECT_EQ(sm->getStateCumulativeTime(State::EXECUTE), 0.0);
  sm->deactivate();
}
