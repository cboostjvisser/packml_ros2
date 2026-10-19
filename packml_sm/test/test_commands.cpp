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
// PackML accept matrix for the *acting* states.  An acting state owns no command edge of its
// own; what it accepts is exactly what it inherits from its ancestors, so the three rows below
// are the three positions a state can occupy in the hierarchy:
//
//   State        | RESET | START | STOP | HOLD | UNHOLD | SUSPEND | UNSUSPEND | ABORT | CLEAR
//   RESETTING    |   .   |   .   |   X  |   .  |    .   |    .    |     .     |   X   |   .
//   STARTING     |   .   |   .   |   X  |   .  |    .   |    .    |     .     |   X   |   .
//   HOLDING      |   .   |   .   |   X  |   .  |    .   |    .    |     .     |   X   |   .
//   UNHOLDING    |   .   |   .   |   X  |   .  |    .   |    .    |     .     |   X   |   .
//   SUSPENDING   |   .   |   .   |   X  |   .  |    .   |    .    |     .     |   X   |   .
//   UNSUSPENDING |   .   |   .   |   X  |   .  |    .   |    .    |     .     |   X   |   .
//   COMPLETING   |   .   |   .   |   X  |   .  |    .   |    .    |     .     |   X   |   .
//   CLEARING     |   .   |   .   |   .  |   .  |    .   |    .    |     .     |   X   |   .
//   STOPPING     |   .   |   .   |   .  |   .  |    .   |    .    |     .     |   X   |   .
//   ABORTING     |   .   |   .   |   .  |   .  |    .   |    .    |     .     |   .   |   .
//
// Stop is valid from any state except ABORTED, ABORTING, CLEARING, STOPPING and STOPPED.
// Abort is valid from any state except ABORTING and ABORTED themselves.  That is why CLEARING
// and STOPPING accept ABORT but refuse STOP, why the other seven acting states accept both, and
// why ABORTING accepts nothing.
//
// The implementation reaches the same answer structurally -- CLEARING and STOPPING hang off
// `abortable` beside `stoppable` rather than inside it -- but the structure is why it happens,
// not why it is right.  ("Abortable" and "Stoppable" are this codebase's identifiers, not
// standard vocabulary.)
//
// ABORTING refuses ABORT for the same reason.  That ABORTING may be ENTERED at any time
// constrains where the abort transition may START; it does not mean a second Abort is honoured
// once the machine is already aborting.  Refusing leaves the in-flight abort undisturbed, where
// a self-transition would restart the rapid-safe-stop sequence mid-run.  A command not valid for
// the current state must not act on the machine but must report itself -- which is why refusal
// is a logged WARN naming the command, not a silent false.
//
// "X" = accepted (returns true); "." = rejected (returns false).
// ---

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <stop_token>
#include <vector>

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
  EXPECT_TRUE(sm->activate());
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

// COMPLETE accepts RESET, STOP, ABORT.
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
    ASSERT_TRUE(sm->activate());
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

// ---------------------------------------------------------------------------
// Acting states.
//
// These were never swept. The sweep above covers seven states and every one of them is a waiting
// state, which is exactly how ABORTING came to answer every command with success: no command
// transition is consulted there, and a QEvent's accept flag is only ever lowered BY a transition.
// Testing what an acting state refuses needs the machine held inside it, so each case binds a
// gated operation to the one state under test and leaves every other state at the default delay.
// The gate is a condition variable the test opens, not a sleep: a timer would race the commands
// and turn a real refusal into a flake, or worse, hide one.
// ---------------------------------------------------------------------------

namespace
{
// Holds an acting state open until the test says otherwise. Released by the test, by the state
// machine asking the operation to stop (which is what an ACCEPTED command triggers), or by a
// safety cap so a wiring mistake cannot wedge the suite.
struct HoldGate
{
  std::mutex open_mutex;
  std::condition_variable open_cv;
  bool open{false};

  void release()
  {
    {
      std::lock_guard<std::mutex> lk(open_mutex);
      open = true;
    }
    open_cv.notify_all();
  }
};

std::shared_ptr<packml_sm::StateMachine> fresh_sm_holding(
  State acting, std::shared_ptr<HoldGate> gate, bool single_cycle = false)
{
  auto sm = single_cycle
    ? packml_sm::StateMachine::singleCycleSM(packml_sm_test::kTestDelayMs)
    : packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  // COMPLETING is only reachable once EXECUTE finishes, so that one case needs EXECUTE to be
  // quick rather than deliberately slow.
  sm->setExecute(single_cycle
    ? std::function<int()>(packml_sm_test::execute_success_fast)
    : std::function<int()>(packml_sm_test::execute_success_long));
  sm->setInterruptibleStateOperation(acting, [gate](std::stop_token tok) -> int {
      const auto cap = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (!tok.stop_requested() && std::chrono::steady_clock::now() < cap) {
        std::unique_lock<std::mutex> lk(gate->open_mutex);
        if (gate->open_cv.wait_for(lk, std::chrono::milliseconds(10), [&] { return gate->open; })) {
          break;
        }
      }
      return 0;
    });
  EXPECT_TRUE(sm->activate());
  return sm;
}

// Drive to `target` and stop there, held by the gated operation bound to it.
bool drive_to_acting(packml_sm::StateMachine & sm, State target)
{
  using namespace std::chrono_literals;
  if (!wait_for_state(sm, State::STOPPED)) return false;
  if (!packml_sm_test::enable_all_transitions(sm)) return false;

  switch (target) {
    case State::ABORTING:
      if (!sm.abort()) return false;
      break;
    case State::CLEARING:
      if (!sm.abort()) return false;
      if (!wait_for_state(sm, State::ABORTED, 5s)) return false;
      if (!sm.clear()) return false;
      break;
    case State::RESETTING:
      if (!sm.reset()) return false;
      break;
    case State::STOPPING:
      if (!sm.reset()) return false;
      if (!wait_for_state(sm, State::IDLE, 5s)) return false;
      if (!sm.stop()) return false;
      break;
    case State::STARTING:
      if (!sm.reset()) return false;
      if (!wait_for_state(sm, State::IDLE, 5s)) return false;
      if (!sm.start()) return false;
      break;
    case State::HOLDING:
    case State::SUSPENDING:
    case State::UNHOLDING:
    case State::UNSUSPENDING:
    case State::COMPLETING:
      if (!sm.reset()) return false;
      if (!wait_for_state(sm, State::IDLE, 5s)) return false;
      if (!sm.start()) return false;
      if (State::COMPLETING == target) {
        break;  // reached by EXECUTE completing, not by a command
      }
      if (!wait_for_state(sm, State::EXECUTE, 5s)) return false;
      if (State::HOLDING == target || State::UNHOLDING == target) {
        if (!sm.hold()) return false;
        if (State::UNHOLDING == target) {
          if (!wait_for_state(sm, State::HELD, 5s)) return false;
          if (!sm.unhold()) return false;
        }
      } else {
        if (!sm.suspend()) return false;
        if (State::UNSUSPENDING == target) {
          if (!wait_for_state(sm, State::SUSPENDED, 5s)) return false;
          if (!sm.unsuspend()) return false;
        }
      }
      break;
    default:
      return false;
  }
  return wait_for_state(sm, target, 5s);
}

// Rejected commands leave the machine where it is, so they all share one visit; each accepted
// command gets its own, because accepting moves the machine out of the state under test.
void check_acting_acceptance(
  State state, std::vector<CommandExpectation> matrix, bool single_cycle = false)
{
  {
    auto gate = std::make_shared<HoldGate>();
    auto sm = fresh_sm_holding(state, gate, single_cycle);
    ASSERT_TRUE(drive_to_acting(*sm, state)) << "Failed to drive to " << state;
    for (const auto & exp : matrix) {
      if (exp.accepted) continue;
      ASSERT_EQ(sm->getCurrentState(), state)
        << "left " << state << " before " << exp.cmd << " was tried";
      EXPECT_FALSE(issue(*sm, exp.cmd))
        << "State " << state << ", command " << exp.cmd << ": expected reject, got accept";
      EXPECT_EQ(sm->getCurrentState(), state)
        << "Rejected " << exp.cmd << " moved SM out of " << state;
    }
    gate->release();
    sm->deactivate();
  }

  for (const auto & exp : matrix) {
    if (!exp.accepted) continue;
    auto gate = std::make_shared<HoldGate>();
    auto sm = fresh_sm_holding(state, gate, single_cycle);
    ASSERT_TRUE(drive_to_acting(*sm, state)) << "Failed to drive to " << state;
    // Without this the gate could have lapsed and the command be accepted by the SUCCESSOR
    // state instead -- STOP is legal from IDLE as well as from RESETTING, so an accept proves
    // nothing until it is known which state answered.
    ASSERT_EQ(sm->getCurrentState(), state)
      << "left " << state << " before " << exp.cmd << " was tried";
    EXPECT_TRUE(issue(*sm, exp.cmd))
      << "State " << state << ", command " << exp.cmd << ": expected accept, got reject";
    gate->release();
    sm->deactivate();
  }
}

// The seven acting states inside `stoppable`: STOP and ABORT, nothing else.
std::vector<CommandExpectation> stoppable_acting_matrix()
{
  return {
    {TransitionCmd::RESET,     false},
    {TransitionCmd::START,     false},
    {TransitionCmd::STOP,      true },
    {TransitionCmd::HOLD,      false},
    {TransitionCmd::UNHOLD,    false},
    {TransitionCmd::SUSPEND,   false},
    {TransitionCmd::UNSUSPEND, false},
    {TransitionCmd::ABORT,     true },
    {TransitionCmd::CLEAR,     false},
  };
}

// CLEARING and STOPPING sit beside `stoppable`, not inside it: ABORT only.
std::vector<CommandExpectation> abortable_only_acting_matrix()
{
  auto matrix = stoppable_acting_matrix();
  for (auto & exp : matrix) {
    if (TransitionCmd::STOP == exp.cmd) exp.accepted = false;
  }
  return matrix;
}
}  // namespace

TEST(CommandMatrix, ResettingAcceptsStopAndAbort)
{
  check_acting_acceptance(State::RESETTING, stoppable_acting_matrix());
}

TEST(CommandMatrix, StartingAcceptsStopAndAbort)
{
  check_acting_acceptance(State::STARTING, stoppable_acting_matrix());
}

TEST(CommandMatrix, HoldingAcceptsStopAndAbort)
{
  check_acting_acceptance(State::HOLDING, stoppable_acting_matrix());
}

TEST(CommandMatrix, UnholdingAcceptsStopAndAbort)
{
  check_acting_acceptance(State::UNHOLDING, stoppable_acting_matrix());
}

TEST(CommandMatrix, SuspendingAcceptsStopAndAbort)
{
  check_acting_acceptance(State::SUSPENDING, stoppable_acting_matrix());
}

TEST(CommandMatrix, UnsuspendingAcceptsStopAndAbort)
{
  check_acting_acceptance(State::UNSUSPENDING, stoppable_acting_matrix());
}

TEST(CommandMatrix, CompletingAcceptsStopAndAbort)
{
  check_acting_acceptance(State::COMPLETING, stoppable_acting_matrix(), /*single_cycle=*/true);
}

TEST(CommandMatrix, ClearingAcceptsOnlyAbort)
{
  check_acting_acceptance(State::CLEARING, abortable_only_acting_matrix());
}

TEST(CommandMatrix, StoppingAcceptsOnlyAbort)
{
  check_acting_acceptance(State::STOPPING, abortable_only_acting_matrix());
}

// ABORTING inherits no command edge from any ancestor, so every one of the nine must be refused
// and the machine must stay in ABORTING. Its only exit is State Complete, to ABORTED.
TEST(CommandMatrix, AbortingAcceptsNothing)
{
  auto matrix = stoppable_acting_matrix();
  for (auto & exp : matrix) {
    exp.accepted = false;
  }
  check_acting_acceptance(State::ABORTING, matrix);
}
