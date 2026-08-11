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
// Error-path tests.  When an Acting state's bound function returns non-zero,
// it posts ErrorEvent which must escalate via Aborting to ABORTED.  PackML
// distinguishes Holding (minor, internal) from Aborted (major faults), but
// the SM's automatic-escalation default is to ABORTED.  Custom OEM logic for
// Holding-on-error is integrator territory and not enforced by the library.
// TODO: work out the error logic
// ---

#include <chrono>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "packml_sm/state_machine.hpp"
#include "test_helpers.hpp"

using packml_sm::State;
using packml_sm_test::wait_for_state;
using packml_sm_test::drive_to_idle;
using packml_sm_test::execute_fail;
using packml_sm_test::execute_success_long;

namespace {

std::string join_names(const std::vector<std::string> & names)
{
  std::string joined;
  for (const auto & name : names) {
    joined += joined.empty() ? name : (", " + name);
  }
  return joined;
}

}  // namespace

// Every acting state must OWN the ERROR transition that reaches its declared failure target,
// rather than inherit one from a superstate.  The two look equivalent and are not: an inherited
// escape serves only descendants, and it silently serves none at all for a state constructed
// outside that superstate.  ABORTING is that state -- parentless, so it inherited nothing, and
// the superstate's error edge targeted ABORTING itself anyway, so inheriting it would have
// self-looped rather than escaped.  A failure of ABORTING's own operation therefore posted an
// event that matched nothing, which Qt discarded, wedging the machine with no reachable exit.
// The other twelve tests in this file only prove that errors escalate today; this one is what
// keeps that true when a thirteenth acting state is added.
TEST(Errors, EveryActingStateOwnsItsErrorEscape)
{
  auto continuous = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  const auto continuous_faults = continuous->errorEscapeReport();
  EXPECT_TRUE(continuous_faults.empty())
    << "continuous-cycle failure-target faults: " << join_names(continuous_faults);

  auto single = packml_sm::StateMachine::singleCycleSM(packml_sm_test::kTestDelayMs);
  const auto single_faults = single->errorEscapeReport();
  EXPECT_TRUE(single_faults.empty())
    << "single-cycle failure-target faults: " << join_names(single_faults);

  // And the report is what activate() gates on, so a graph that fails it cannot be run at all.
  EXPECT_TRUE(continuous->activate());
  continuous->deactivate();
  EXPECT_TRUE(single->activate());
  single->deactivate();
}

TEST(Errors, ExecuteErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  sm->deactivate();
}

TEST(Errors, ResettingErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->setResetting(std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  ASSERT_TRUE(packml_sm_test::enable_all_transitions(*sm));
  ASSERT_TRUE(sm->reset());
  // RESETTING fails -> error event -> machine must end up in ABORTED.
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  sm->deactivate();
}

// PackML: error during EXECUTE does not directly transition to HOLDING; the
// library only triggers the abort path.  Document that contract here.
TEST(Errors, ExecuteErrorDoesNotTransitionToHolding)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  // We expect ABORTED, not HELD.
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  EXPECT_NE(sm->getCurrentState(), State::HELD);
  sm->deactivate();
}

// Error code from the failing function must be observable via getLastErrorCode().
TEST(Errors, ErrorCodeIsObservable)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute([] { return 42; });
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  EXPECT_EQ(sm->getLastErrorCode(), 42);
  sm->deactivate();
}

// Error in STARTING acting state must also escalate to ABORTED.
TEST(Errors, StartingErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->setStateOperation(State::STARTING, std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  sm->deactivate();
}

// Error in HOLDING acting state must escalate to ABORTED.
TEST(Errors, HoldingErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->setStateOperation(State::HOLDING, std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  ASSERT_TRUE(sm->hold());
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  sm->deactivate();
}

// Error in STOPPING acting state must escalate to ABORTED.
TEST(Errors, StoppingErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->setStateOperation(State::STOPPING, std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  ASSERT_TRUE(sm->stop());
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  sm->deactivate();
}

// Error in SUSPENDING acting state must escalate to ABORTED.
TEST(Errors, SuspendingErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->setStateOperation(State::SUSPENDING, std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  ASSERT_TRUE(sm->suspend());
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  sm->deactivate();
}

// Error in UNSUSPENDING acting state must escalate to ABORTED.
TEST(Errors, UnsuspendingErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->setStateOperation(State::UNSUSPENDING, std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  ASSERT_TRUE(sm->suspend());
  ASSERT_TRUE(wait_for_state(*sm, State::SUSPENDED, std::chrono::seconds(5)));
  ASSERT_TRUE(sm->unsuspend());
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  sm->deactivate();
}

// Error in UNHOLDING acting state must escalate to ABORTED.
TEST(Errors, UnholdingErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->setStateOperation(State::UNHOLDING, std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  ASSERT_TRUE(sm->hold());
  ASSERT_TRUE(wait_for_state(*sm, State::HELD, std::chrono::seconds(5)));
  ASSERT_TRUE(sm->unhold());
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  sm->deactivate();
}

// Error in COMPLETING acting state must escalate to ABORTED.
TEST(Errors, CompletingErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::singleCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->setStateOperation(State::COMPLETING, std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  // Single-cycle: EXECUTE → COMPLETING (which fails) → ABORTED
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  sm->deactivate();
}

// Error in the ABORTING acting state must still land somewhere -- ABORTED, the safe terminal.
//
// ABORTING was the one acting state of the eleven with no failing-operation test, and that gap is
// why its missing error escape shipped: the machine had no exit at all from a failed abort, since
// ABORTING's only other out-edge is its own STATE_COMPLETED, which the failed operation never
// posts. Not a theoretical state to fail in either -- packml_ros binds ABORTING as a coordinated
// state whose operation returns non-zero whenever the fan-out to child nodes times out, which is
// precisely what happens when the node that caused the abort is the one that stopped answering.
TEST(Errors, AbortingErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->setStateOperation(State::ABORTING, std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->abort());
  // ABORTING's own operation fails -> its declared failure target is the terminal ABORTED.
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  // And the machine is genuinely recoverable from there, not merely reporting ABORTED.
  ASSERT_TRUE(sm->clear());
  EXPECT_TRUE(wait_for_state(*sm, State::STOPPED, std::chrono::seconds(5)));
  sm->deactivate();
}

// Error in CLEARING acting state must escalate to ABORTED.
TEST(Errors, ClearingErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->setStateOperation(State::CLEARING, std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  ASSERT_TRUE(packml_sm_test::enable_all_transitions(*sm));
  // Navigate to ABORTED first so we can issue clear().
  ASSERT_TRUE(sm->abort());
  ASSERT_TRUE(wait_for_state(*sm, State::ABORTED));
  ASSERT_TRUE(sm->clear());
  // CLEARING fails → error → back to ABORTED.
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)));
  sm->deactivate();
}
