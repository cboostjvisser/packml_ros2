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
// Command-rejection matrix.  PackML defines exactly which commands are valid
// in each waiting state (acting states reject all commands except those that
// transition into another acting state via the abortable/stoppable
// supergroups).  This file pins the contract by sending every command from
// every reachable waiting state and asserting accept/reject.
//
// PackML accept matrix for the *waiting* states:
//
//   State      | RESET | START | STOP | HOLD | UNHOLD | SUSPEND | UNSUSPEND | ABORT | CLEAR
//   ABORTED    |   .   |   .   |   .  |   .  |    .   |    .    |     .     |   .   |   X
//   STOPPED    |   X   |   .   |   .  |   .  |    .   |    .    |     .     |   X   |   .
//   IDLE       |   .   |   X   |   X  |   .  |    .   |    .    |     .     |   X   |   .
//   EXECUTE    |   .   |   .   |   X  |   X  |    .   |    X    |     .     |   X   |   .
//   HELD       |   .   |   .   |   X  |   .  |    X   |    .    |     .     |   X   |   .
//   SUSPENDED  |   .   |   .   |   X  |   .  |    .   |    .    |     X     |   X   |   .
//   COMPLETE   |   X   |   .   |   X  |   .  |    .   |    .    |     .     |   X   |   .
//
// "X" = accepted (returns true); "." = rejected (returns false).
// ---

#include <chrono>
#include <gtest/gtest.h>

#include "packml_sm/state_machine.hpp"
#include "test_helpers.hpp"

using packml_sm::State;
using packml_sm::TransitionCmd;
using packml_sm_test::wait_for_state;
using packml_sm_test::execute_success_long;
using packml_sm_test::drive_to_idle;

namespace
{
// Issue a command via the public command method matching `cmd` and return the
// boolean result.  Routes through the synchronous `_X()` path which blocks
// until the Qt machine has accepted/rejected.
bool issue(packml_sm::StateMachine & sm, TransitionCmd cmd)
{
  switch (cmd) {
    case TransitionCmd::RESET:     return sm.reset();
    case TransitionCmd::START:     return sm.start();
    case TransitionCmd::STOP:      return sm.stop();
    case TransitionCmd::HOLD:      return sm.hold();
    case TransitionCmd::UNHOLD:    return sm.unhold();
    case TransitionCmd::SUSPEND:   return sm.suspend();
    case TransitionCmd::UNSUSPEND: return sm.unsuspend();
    case TransitionCmd::ABORT:     return sm.abort();
    case TransitionCmd::CLEAR:     return sm.clear();
    case TransitionCmd::NO_COMMAND: return false;
  }
  return false;
}

// Drive a continuous-cycle SM to the requested *waiting* state.
bool drive_to(packml_sm::StateMachine & sm, State target)
{
  using namespace std::chrono_literals;
  // SM boots in STOPPED per PackML.
  if (!wait_for_state(sm, State::STOPPED)) return false;
  if (!packml_sm_test::enable_all_transitions(sm)) return false;

  if (target == State::ABORTED) {
    if (!sm.abort()) return false;
    return wait_for_state(sm, State::ABORTED);
  }
  if (target == State::STOPPED) return true;

  if (!sm.reset()) return false;
  if (!wait_for_state(sm, State::IDLE)) return false;
  if (target == State::IDLE) return true;

  if (!sm.start()) return false;
  if (!wait_for_state(sm, State::EXECUTE)) return false;
  if (target == State::EXECUTE) return true;

  if (target == State::HELD) {
    if (!sm.hold()) return false;
    return wait_for_state(sm, State::HELD, 5s);
  }
  if (target == State::SUSPENDED) {
    if (!sm.suspend()) return false;
    return wait_for_state(sm, State::SUSPENDED, 5s);
  }
  if (target == State::COMPLETE) {
    // Need a single-cycle SM to reach COMPLETE; for continuous-cycle we
    // can't use this path.  Caller must provide a single-cycle SM.
    return wait_for_state(sm, State::COMPLETE, 5s);
  }
  return false;
}

// Build a fresh continuous-cycle SM (so EXECUTE doesn't auto-complete).
auto fresh_sm()
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->activate();
  return sm;
}

struct CommandExpectation
{
  TransitionCmd cmd;
  bool accepted;
};

// Drive `sm` to `state`, then for each (cmd, expected) issue the command and
// expect the boolean return matches.  After issuing rejected commands, the SM
// must remain in `state`.  Accepted commands move the SM, so the test
// re-drives between checks where needed.
void check_acceptance(State state, std::vector<CommandExpectation> matrix)
{
  for (const auto & exp : matrix) {
    auto sm = fresh_sm();
    ASSERT_TRUE(drive_to(*sm, state)) << "Failed to drive to " << state;
    const bool got = issue(*sm, exp.cmd);
    EXPECT_EQ(got, exp.accepted)
      << "State " << state << ", command " << exp.cmd
      << ": expected " << (exp.accepted ? "accept" : "reject")
      << ", got " << (got ? "accept" : "reject");
    if (!exp.accepted) {
      // Rejected commands must not change state.
      EXPECT_EQ(sm->getCurrentState(), state)
        << "Rejected " << exp.cmd << " moved SM out of " << state;
    }
    sm->deactivate();
  }
}
}  // namespace

TEST(CommandMatrix, AbortedAcceptsOnlyClear)
{
  check_acceptance(State::ABORTED, {
    {TransitionCmd::RESET,     false},
    {TransitionCmd::START,     false},
    {TransitionCmd::STOP,      false},
    {TransitionCmd::HOLD,      false},
    {TransitionCmd::UNHOLD,    false},
    {TransitionCmd::SUSPEND,   false},
    {TransitionCmd::UNSUSPEND, false},
    {TransitionCmd::ABORT,     false},
    {TransitionCmd::CLEAR,     true },
  });
}

TEST(CommandMatrix, StoppedAcceptsResetAndAbort)
{
  check_acceptance(State::STOPPED, {
    {TransitionCmd::RESET,     true },
    {TransitionCmd::START,     false},
    {TransitionCmd::STOP,      false},
    {TransitionCmd::HOLD,      false},
    {TransitionCmd::UNHOLD,    false},
    {TransitionCmd::SUSPEND,   false},
    {TransitionCmd::UNSUSPEND, false},
    {TransitionCmd::ABORT,     true },
    {TransitionCmd::CLEAR,     false},
  });
}

TEST(CommandMatrix, IdleAcceptsStartStopAbort)
{
  check_acceptance(State::IDLE, {
    {TransitionCmd::RESET,     false},
    {TransitionCmd::START,     true },
    {TransitionCmd::STOP,      true },
    {TransitionCmd::HOLD,      false},
    {TransitionCmd::UNHOLD,    false},
    {TransitionCmd::SUSPEND,   false},
    {TransitionCmd::UNSUSPEND, false},
    {TransitionCmd::ABORT,     true },
    {TransitionCmd::CLEAR,     false},
  });
}

TEST(CommandMatrix, ExecuteAcceptsHoldSuspendStopAbort)
{
  check_acceptance(State::EXECUTE, {
    {TransitionCmd::RESET,     false},
    {TransitionCmd::START,     false},
    {TransitionCmd::STOP,      true },
    {TransitionCmd::HOLD,      true },
    {TransitionCmd::UNHOLD,    false},
    {TransitionCmd::SUSPEND,   true },
    {TransitionCmd::UNSUSPEND, false},
    {TransitionCmd::ABORT,     true },
    {TransitionCmd::CLEAR,     false},
  });
}

TEST(CommandMatrix, HeldAcceptsUnholdStopAbort)
{
  check_acceptance(State::HELD, {
    {TransitionCmd::RESET,     false},
    {TransitionCmd::START,     false},
    {TransitionCmd::STOP,      true },
    {TransitionCmd::HOLD,      false},
    {TransitionCmd::UNHOLD,    true },
    {TransitionCmd::SUSPEND,   false},
    {TransitionCmd::UNSUSPEND, false},
    {TransitionCmd::ABORT,     true },
    {TransitionCmd::CLEAR,     false},
  });
}

TEST(CommandMatrix, SuspendedAcceptsUnsuspendStopAbort)
{
  check_acceptance(State::SUSPENDED, {
    {TransitionCmd::RESET,     false},
    {TransitionCmd::START,     false},
    {TransitionCmd::STOP,      true },
    {TransitionCmd::HOLD,      false},
    {TransitionCmd::UNHOLD,    false},
    {TransitionCmd::SUSPEND,   false},
    {TransitionCmd::UNSUSPEND, true },
    {TransitionCmd::ABORT,     true },
    {TransitionCmd::CLEAR,     false},
  });
}

// COMPLETE accepts RESET, STOP, ABORT per PackML spec.
// Requires a single-cycle SM to reach COMPLETE naturally.
TEST(CommandMatrix, CompleteAcceptsResetStopAbort)
{
  using namespace std::chrono_literals;
  // Per-command check with a fresh single-cycle SM each time.
  std::vector<CommandExpectation> matrix = {
    {TransitionCmd::RESET,     true },
    {TransitionCmd::START,     false},
    {TransitionCmd::STOP,      true },
    {TransitionCmd::HOLD,      false},
    {TransitionCmd::UNHOLD,    false},
    {TransitionCmd::SUSPEND,   false},
    {TransitionCmd::UNSUSPEND, false},
    {TransitionCmd::ABORT,     true },
    {TransitionCmd::CLEAR,     false},
  };
  for (const auto & exp : matrix) {
    auto sm = packml_sm::StateMachine::singleCycleSM(packml_sm_test::kTestDelayMs);
    sm->setExecute(std::bind(execute_success_long));
    sm->activate();
    ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
    ASSERT_TRUE(packml_sm_test::enable_all_transitions(*sm));
    ASSERT_TRUE(sm->reset());
    ASSERT_TRUE(wait_for_state(*sm, State::IDLE));
    ASSERT_TRUE(sm->start());
    ASSERT_TRUE(wait_for_state(*sm, State::COMPLETE, 5s));
    const bool got = issue(*sm, exp.cmd);
    EXPECT_EQ(got, exp.accepted)
      << "State COMPLETE, command " << exp.cmd
      << ": expected " << (exp.accepted ? "accept" : "reject")
      << ", got " << (got ? "accept" : "reject");
    sm->deactivate();
  }
}
