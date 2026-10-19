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
// The bridge between a caller's thread and the Qt event loop: the nine command methods post a
// CmdEvent carrying a promise and wait for transition selection to fulfil it.
//
// Everything here is about what happens when that answer cannot arrive, or arrives wrong. Those
// cases were all silent: a command to a machine that was not running blocked its caller for the
// life of the process (Qt discards AND leaks such an event, so the promise is neither fulfilled
// nor destroyed), and a command in a state that consults no command transition reported success
// while the machine stood still.
//
// A hang cannot be asserted with EXPECT -- a wedged test takes the whole binary to the ctest
// timeout with no useful output. So every "does not block" test runs the command on a detached
// thread that reports through its own promise, and the test asserts that the report ARRIVED.
// std::async is deliberately not used: its returned future blocks in its own destructor, which
// would re-introduce the hang inside the test harness itself.
// ---

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <optional>
#include <memory>
#include <string>
#include <thread>

#include <QCoreApplication>
#include <gtest/gtest.h>

#include "packml_sm/common.hpp"
#include "packml_sm/state_machine.hpp"
#include "test_helpers.hpp"

using namespace std::chrono_literals;
using packml_sm::State;
using packml_sm::StateMachine;

namespace {

// Run `call` on a detached thread and return its result if it finished within `cap`.
// An empty optional means the call never returned -- the failure this file exists to catch.
std::optional<bool> answered_within(std::function<bool()> call, std::chrono::milliseconds cap)
{
  auto reported = std::make_shared<std::promise<bool>>();
  auto arrived = reported->get_future();
  // Detached, and the promise is shared: if the call never returns, the thread leaks for the rest
  // of the test binary rather than blocking this test's own teardown.
  std::thread([call = std::move(call), reported]() { reported->set_value(call()); }).detach();

  if (arrived.wait_for(cap) != std::future_status::ready) {
    return std::nullopt;
  }
  return arrived.get();
}

}  // namespace

// A command issued before activate() must be refused rather than left to block its caller forever:
// Qt refuses to queue an event on a machine that is not running, but it neither deletes the event
// nor reports the refusal anywhere the caller can see, so the promise inside it would never be
// fulfilled and never destroyed.
TEST(CommandBridge, CommandBeforeActivateIsRefusedNotBlocked)
{
  auto sm = StateMachine::continuousCycleSM(50);

  const auto answer = answered_within([sm]() { return sm->reset(); }, 3s);
  ASSERT_TRUE(answer.has_value())
    << "reset() on a machine that was never activated did not return";
  EXPECT_FALSE(*answer) << "a command the machine never saw must not report success";
}

// The same hazard on the other side of the lifecycle. deactivate() is called in normal shutdown,
// and any command racing it lands here.
TEST(CommandBridge, CommandAfterDeactivateIsRefusedNotBlocked)
{
  auto sm = StateMachine::continuousCycleSM(50);
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::wait_for_state(*sm, State::STOPPED, 3s));
  sm->deactivate();

  const auto answer = answered_within([sm]() { return sm->abort(); }, 3s);
  ASSERT_TRUE(answer.has_value())
    << "abort() on a deactivated machine did not return";
  EXPECT_FALSE(*answer) << "a command the machine never saw must not report success";
}

// Only the Qt event loop fulfils a command's promise, so a caller running ON that loop is waiting
// for a reply that only it could send. This is reachable through public API: on_state_changed and
// on_error_code are assignable members invoked from the loop thread.
//
// On regression this test HANGS rather than fails: the callback never returns, so the Qt thread
// stays inside it and teardown blocks. Read a ctest timeout on this test as a failure of it.
//
// Everything the callback touches is held by shared_ptr and captured by value, and the callback is
// never reassigned. Both matter only on regression, and for the same reason: the closure is then
// still executing when this frame returns, so anything it captured by reference would dangle and
// replacing it would free a std::function whose operator() is mid-call. Either turns the
// documented hang into a segfault inside whichever test runs next, which is a far worse way to
// learn that this guard has gone.
TEST(CommandBridge, CommandFromTheQtThreadIsRefusedNotDeadlocked)
{
  struct Observed
  {
    std::atomic<bool> ran{false};
    std::atomic<bool> result{true};
  };
  auto observed = std::make_shared<Observed>();

  auto sm = StateMachine::continuousCycleSM(50);
  sm->on_state_changed = [observed, weak = std::weak_ptr<StateMachine>(sm)](State, QString) {
    if (observed->ran.exchange(true)) {
      return;  // first entry only; this runs on every state change
    }
    if (auto live = weak.lock()) {
      observed->result.store(live->abort());
    }
  };

  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::wait_for_state(*sm, State::STOPPED, 3s));
  sm->deactivate();

  ASSERT_TRUE(observed->ran.load()) << "the callback never ran, so nothing was proven";
  EXPECT_FALSE(observed->result.load())
    << "a command issued from the Qt event-loop thread must be refused, not waited on";
}

// A bound operation must not block on the machine. Neither of the two guards above catches this
// one: the machine is running, and a thread-pool worker is not the loop thread.
//
// This does not deadlock: onExit() does not join the worker on the Qt thread, so the loop is
// free to answer. What it would still do is hold run_mutex_ for the length of the wait, starving
// every later entry to the same state, and it would do it while a correct alternative exists
// (return non-zero and let the ErrorEvent escalate). So the refusal is asserted directly, and the
// resulting state deliberately is not: without the guard the in-operation ABORT is simply accepted,
// and demanding a particular state would report the regression as "wrong state" rather than as the
// missing refusal it is.
TEST(CommandBridge, CommandFromInsideABoundOperationIsRefused)
{
  auto sm = StateMachine::continuousCycleSM(50);

  struct Observed
  {
    std::atomic<bool> ran{false};
    std::atomic<bool> result{true};
  };
  auto observed = std::make_shared<Observed>();

  sm->setStateOperation(State::CLEARING,
    [observed, weak = std::weak_ptr<StateMachine>(sm)]() -> int {
      observed->ran.store(true);
      if (auto live = weak.lock()) {
        observed->result.store(live->abort());
      }
      return 0;
    });

  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::wait_for_state(*sm, State::STOPPED, 3s));

  // CLEARING is not on the power-on path; it is only reached by CLEAR from ABORTED.
  ASSERT_TRUE(sm->abort());
  ASSERT_TRUE(packml_sm_test::wait_for_state(*sm, State::ABORTED, 5s));
  ASSERT_TRUE(sm->clear());

  // Whatever the command did, CLEARING's operation has run by the time the machine settles
  // anywhere. Waiting on the guard's own evidence rather than on a state keeps the assertions
  // below the ones that report a regression.
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!observed->ran.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(5ms);
  }
  sm->deactivate();

  ASSERT_TRUE(observed->ran.load()) << "CLEARING's operation never ran, so nothing was proven";
  EXPECT_FALSE(observed->result.load())
    << "a command issued from inside a bound operation must be refused, not waited on";
}

// A command that no transition claims must report failure, and reporting it needs more than the
// QEvent's own accepted flag: a QEvent is born accepted, Qt never clears that flag, and only a
// transition's eventTest() calling ignore() lowers it -- so in a state where no command transition
// is consulted at all, the flag survives untouched and every command looks accepted.
//
// ABORTING is that state: it hangs off the machine rather than off `abortable`, so the only edges
// consulted are its own completion and error edges, and both return on the event-type check.
TEST(CommandBridge, CommandNoTransitionClaimsReportsFailure)
{
  auto sm = StateMachine::continuousCycleSM(50);
  // Long enough to issue commands while the machine is still inside ABORTING.
  sm->setStateOperation(State::ABORTING, []() -> int {
    std::this_thread::sleep_for(1500ms);
    return 0;
  });
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::wait_for_state(*sm, State::STOPPED, 3s));

  ASSERT_TRUE(sm->abort());
  ASSERT_TRUE(packml_sm_test::wait_for_state(*sm, State::ABORTING, 3s));

  // ABORTING has no command edge at all, so each of these is a command the machine cannot act on.
  EXPECT_FALSE(sm->start()) << "START reported success from ABORTING";
  EXPECT_FALSE(sm->clear()) << "CLEAR reported success from ABORTING";
  EXPECT_FALSE(sm->stop()) << "STOP reported success from ABORTING";
  EXPECT_EQ(sm->getCurrentState(), State::ABORTING)
    << "a refused command moved the machine";

  sm->deactivate();
}

// A command behind a slow bound operation must be answered PROMPTLY.
//
// Joining the bound-operation worker on the Qt thread from ActingState::onExit() would freeze the
// event loop for the operation's whole duration, so a command queued behind a slow operation would
// wait it out instead of being answered. Nothing may reintroduce that join.
//
// A wall-clock deadline in the bridge is the wrong answer too, and this is the test that would
// catch one being added -- a deadline short enough to matter would have to be under the bound
// below, tight enough to fire on ordinary latency.
TEST(CommandBridge, SlowBoundOperationDoesNotDelayACommand)
{
  auto sm = StateMachine::continuousCycleSM(50);
  sm->setStateOperation(State::EXECUTE, []() -> int {
    std::this_thread::sleep_for(3s);
    return 0;
  });
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::wait_for_state(*sm, State::STOPPED, 3s));
  ASSERT_TRUE(sm->reset());
  ASSERT_TRUE(packml_sm_test::wait_for_state(*sm, State::IDLE, 5s));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(packml_sm_test::wait_for_state(*sm, State::EXECUTE, 5s));

  // Let EXECUTE's operation get under way, then interrupt it. This one was always fast: transition
  // selection answers it before the state is exited at all.
  std::this_thread::sleep_for(300ms);
  ASSERT_TRUE(sm->stop());

  // The one that matters: issued into a loop that is now exiting EXECUTE with its 3 s worker still
  // running. ABORT is valid from everywhere in the Abortable superstate.
  const auto began = std::chrono::steady_clock::now();
  const bool accepted = sm->abort();
  const auto waited_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - began).count();

  EXPECT_TRUE(accepted) << "ABORT behind a slow bound operation was refused after "
                        << waited_ms << " ms";
  EXPECT_LT(waited_ms, 500)
    << "ABORT took " << waited_ms << " ms, which tracks the bound operation rather than the event "
       "loop -- something is joining a bound-operation worker on the Qt thread again";

  sm->deactivate();
}

// The fourth way an answer can never arrive, and the only one not reachable by a pre-post guard:
// the command was admitted to a running machine and was still queued when that machine stopped.
// Qt breaks out of its processing loop without draining the queue, so nothing will ever evaluate
// the event. Covered by the wait loop's liveness re-read rather than by a guard -- delete that
// loop and this is the test that notices.
TEST(CommandBridge, CommandStrandedByAStoppingMachineIsRefusedNotBlocked)
{
  auto sm = StateMachine::continuousCycleSM(50);

  // Holds the Qt event loop from the Qt thread itself. A bound operation cannot be used for this,
  // since nothing joins one on that thread, but on_state_changed still runs there, so
  // it is both the honest way to stall the loop in a test and the residual risk a real integrator
  // carries if they do slow work in that callback.
  auto held = std::make_shared<std::atomic<bool>>(false);
  sm->on_state_changed = [held](State value, QString) {
    if (State::IDLE == value && !held->exchange(true)) {
      std::this_thread::sleep_for(1500ms);
    }
  };

  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::wait_for_state(*sm, State::STOPPED, 3s));
  ASSERT_TRUE(sm->reset());
  // Entering IDLE parks the loop inside the callback above.
  ASSERT_TRUE(packml_sm_test::wait_for_state(*sm, State::IDLE, 5s));

  // Queued against a running machine, so it clears every guard -- and then stranded, because the
  // machine stops before the loop is free to look at it.
  auto answer = std::make_shared<std::promise<bool>>();
  auto arrived = answer->get_future();
  std::thread([sm, answer]() { answer->set_value(sm->start()); }).detach();

  std::this_thread::sleep_for(100ms);
  sm->deactivate();

  ASSERT_EQ(arrived.wait_for(5s), std::future_status::ready)
    << "a command stranded by a stopping machine never returned";
  EXPECT_FALSE(arrived.get()) << "a command that was never evaluated must not report success";
}
