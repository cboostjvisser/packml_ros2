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
// Lifecycle tests: factory creation, activation/deactivation, isActive(), and
// the high-level "follow the diagram" happy paths for SingleCycle and
// ContinuousCycle.
// ---

#include <chrono>
#include <atomic>
#include <gtest/gtest.h>
#include <memory>
#include <stop_token>
#include <thread>

#include <QThreadPool>
#include <QtConcurrent/QtConcurrent>

#include "packml_sm/state_machine.hpp"
#include "test_helpers.hpp"

using packml_sm::State;
using packml_sm_test::wait_for_state;
using packml_sm_test::execute_success_long;
using packml_sm_test::execute_fail;

TEST(Lifecycle, SingleCycleFactoryProducesActivatableSM)
{
  auto sm = packml_sm::StateMachine::singleCycleSM(packml_sm_test::kTestDelayMs);
  ASSERT_NE(sm, nullptr);
  EXPECT_FALSE(sm->isActive());
  sm->setExecute(std::bind(execute_success_long));
  ASSERT_TRUE(sm->activate());
  EXPECT_TRUE(wait_for_state(*sm, State::STOPPED))
    << "PackML cold-start should land in STOPPED per PackML.";
  EXPECT_TRUE(sm->isActive());
  sm->deactivate();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_FALSE(sm->isActive());
}

TEST(Lifecycle, ContinuousCycleFactoryProducesActivatableSM)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  ASSERT_NE(sm, nullptr);
  EXPECT_FALSE(sm->isActive());
  sm->setExecute(std::bind(execute_success_long));
  ASSERT_TRUE(sm->activate());
  EXPECT_TRUE(wait_for_state(*sm, State::STOPPED));
  sm->deactivate();
}

// PackML happy-path single cycle:
//   STOPPED -RESET-> RESETTING -SC-> IDLE
//   -START-> STARTING -SC-> EXECUTE -SC-> COMPLETING -SC-> COMPLETE
TEST(Lifecycle, SingleCycleFollowsHappyPathToComplete)
{
  auto sm = packml_sm::StateMachine::singleCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  ASSERT_TRUE(packml_sm_test::enable_all_transitions(*sm));

  ASSERT_TRUE(sm->reset());
  EXPECT_TRUE(wait_for_state(*sm, State::IDLE));

  ASSERT_TRUE(sm->start());
  // Single-cycle: execute completes once, machine progresses to COMPLETE.
  EXPECT_TRUE(wait_for_state(*sm, State::COMPLETE, std::chrono::seconds(5)));

  // From COMPLETE, RESET returns to IDLE.
  ASSERT_TRUE(sm->reset());
  EXPECT_TRUE(wait_for_state(*sm, State::IDLE));

  sm->deactivate();
}

// PackML continuous cycle: EXECUTE loops; only HOLD/SUSPEND/STOP/ABORT can
// pull the machine out of EXECUTE.
TEST(Lifecycle, ContinuousCycleStaysInExecuteUntilCommanded)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  ASSERT_TRUE(packml_sm_test::enable_all_transitions(*sm));
  ASSERT_TRUE(sm->reset());
  ASSERT_TRUE(wait_for_state(*sm, State::IDLE));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));

  // It should NOT progress to COMPLETE on its own.
  EXPECT_FALSE(wait_for_state(*sm, State::COMPLETE, std::chrono::seconds(2)));

  ASSERT_TRUE(sm->stop());
  EXPECT_TRUE(wait_for_state(*sm, State::STOPPED));
  sm->deactivate();
}

// Failure of execute callback must escalate to ABORTED via ErrorEvent.
TEST(Lifecycle, ExecuteErrorEscalatesToAborted)
{
  auto sm = packml_sm::StateMachine::singleCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_fail));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  ASSERT_TRUE(packml_sm_test::enable_all_transitions(*sm));
  ASSERT_TRUE(sm->reset());
  ASSERT_TRUE(wait_for_state(*sm, State::IDLE));
  ASSERT_TRUE(sm->start());
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)))
    << "An execute() returning non-zero must trigger ABORTING -> ABORTED.";
  sm->deactivate();
}

// Robustness: multiple activate/deactivate cycles using fresh SM instances.
// NOTE: Re-activating the SAME QStateMachine instance after stop() is a known
// Qt5 limitation (the internal state graph does not fully reset).  The
// production-safe pattern is to create a new SM for each activation cycle.
TEST(Lifecycle, MultipleActivateDeactivateCycles)
{
  for (int i = 0; i < 3; ++i) {
    auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
    sm->setExecute(std::bind(execute_success_long));
    ASSERT_TRUE(sm->activate()) << "activate() failed on cycle " << i;
    ASSERT_TRUE(wait_for_state(*sm, State::STOPPED))
      << "Did not reach STOPPED on cycle " << i;
    // Drive to IDLE to prove the SM is fully operational.
    ASSERT_TRUE(packml_sm_test::enable_all_transitions(*sm));
    ASSERT_TRUE(sm->reset());
    ASSERT_TRUE(wait_for_state(*sm, State::IDLE))
      << "Did not reach IDLE on cycle " << i;
    sm->deactivate();
    // Give Qt event loop time to fully shut down.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

// Robustness: deactivate() while an acting state is mid-flight (long operation)
// must terminate cleanly without SIGSEGV.  The drainActingStates() mechanism
// should wait for the running future to complete.
TEST(Lifecycle, DeactivateDuringActingStateIsClean)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  // Use a 200 ms execute so it's definitely still running when we deactivate.
  sm->setExecute([] {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    return 0;
  });
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  // Immediately deactivate while EXECUTE's operation is still sleeping.
  sm->deactivate();
  // If we reach here without crashing, the test passes.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_FALSE(sm->isActive());
}

// activate() must not return until the machine is genuinely running.
//
// QStateMachine::start() only QUEUES the start onto the Qt loop, so returning before that lands
// reports true while isRunning() is still false. Everything that decides teardown branches on it:
// deactivate() takes an early return that skips stop() altogether, and ~StateMachine() picks
// between deactivating and merely draining -- so a caller doing the entirely reasonable
// activate()-then-deactivate() could tear the machine down while the Qt loop was still about to
// start it and walk a state graph being freed underneath it.
//
// Asserted with no wait of any kind after activate(), which is the whole point: a test that slept
// first would pass either way.
TEST(Lifecycle, ActivateReturnsOnlyOnceTheMachineIsRunning)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  ASSERT_TRUE(sm->activate());
  EXPECT_TRUE(sm->isActive())
    << "activate() returned true while the machine was not running yet, so every isRunning() "
       "check downstream -- including the two that decide how the machine is torn down -- is "
       "reading a value that is about to change on its own";
  sm->deactivate();
}

// QStateMachine::stop() does not call onExit() on active states, so deactivate() has to make the
// stop request onExit() makes everywhere else.  Before it did, an in-flight bound operation was
// never told the machine was going away: it ran to its own natural end and then posted its
// StateCompleteEvent into a stopped machine, which Qt warns about, drops, and does not delete.
//
// Both assertions are consequences of that request now being made -- the operation observes its own
// token, and deactivate() therefore returns in a fraction of the operation's remaining duration
// instead of waiting it out.
TEST(Lifecycle, DeactivateAsksTheBoundOperationToStop)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);

  struct Observed
  {
    std::atomic<bool> running{false};
    std::atomic<bool> stop_requested{false};
  };
  auto observed = std::make_shared<Observed>();

  // Three seconds of "work" that gives up the moment it is asked to, so waiting it out is
  // unmistakable in the elapsed time below rather than a matter of a few milliseconds.
  constexpr auto kOperation = std::chrono::seconds(3);
  ASSERT_TRUE(sm->setInterruptibleStateOperation(State::EXECUTE,
    [observed, kOperation](std::stop_token token) -> int {
      observed->running.store(true);
      const auto deadline = std::chrono::steady_clock::now() + kOperation;
      while (!token.stop_requested() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      observed->stop_requested.store(token.stop_requested());
      return 0;
    }));

  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));

  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!observed->running.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
  ASSERT_TRUE(observed->running.load())
    << "EXECUTE's operation never started, so nothing below is proven";

  const auto t0 = std::chrono::steady_clock::now();
  sm->deactivate();
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  // The worker stores this before it returns, and returning is what drainActingStates() -- and so
  // deactivate() -- waits on, which is why it is readable here without a further wait.
  EXPECT_TRUE(observed->stop_requested.load())
    << "the operation ran to its own end without ever being told the machine was stopping";
  EXPECT_LT(elapsed, std::chrono::milliseconds(1500))
    << "deactivate() waited out the operation instead of asking it to stop; took "
    << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms";
}

// Robustness: concurrent commands from multiple threads must not deadlock or
// corrupt state.  The per-call promise pattern ensures thread safety.
TEST(Lifecycle, ConcurrentCommandsDoNotCorruptState)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));

  // Fire commands from two threads simultaneously.
  std::atomic<int> completed{0};
  auto fire_commands = [&](int seed) {
    for (int i = 0; i < 50; ++i) {
      switch ((i + seed) % 5) {
        case 0: sm->hold(); break;
        case 1: sm->unhold(); break;
        case 2: sm->suspend(); break;
        case 3: sm->unsuspend(); break;
        case 4: sm->stop(); break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    completed.fetch_add(1);
  };

  std::thread t1(fire_commands, 0);
  std::thread t2(fire_commands, 2);
  t1.join();
  t2.join();

  EXPECT_EQ(completed.load(), 2);

  // The SM must be in a valid PackML state (not crashed/hung).
  auto final_state = sm->getCurrentState();
  // Valid terminal states after this barrage: EXECUTE, HELD, SUSPENDED, STOPPED
  // (or intermediate acting states which will resolve).
  // Wait a moment for any in-flight transitions to complete.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  final_state = sm->getCurrentState();
  bool valid = (final_state == State::EXECUTE ||
                final_state == State::HELD ||
                final_state == State::SUSPENDED ||
                final_state == State::STOPPED ||
                final_state == State::IDLE ||
                final_state == State::ABORTED);
  EXPECT_TRUE(valid)
    << "SM ended in unexpected state: " << static_cast<int>(final_state);
  sm->deactivate();
}

// ~StateMachine() has to wait for its acting states' bound operations to finish, and must not do
// that with QThreadPool::globalInstance()->waitForDone(-1) -- which waits for EVERY QtConcurrent
// user in the address space, not just this machine's. Teardown's duration would then depend on who
// else happened to share the process: another StateMachine's operation, or any library at all.
//
// Parking one unrelated task on the global pool and then destroying a machine is the whole test:
// with the machine owning its own pool the destructor cannot see that task, and without it the
// destructor waits the task out.
TEST(Lifecycle, DestructorDoesNotWaitForUnrelatedThreadPoolWork)
{
  std::atomic<bool> release{false};
  std::atomic<bool> parked_started{false};
  QtConcurrent::run(
    QThreadPool::globalInstance(),
    [&release, &parked_started]() {
      parked_started.store(true);
      while (!release.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    });

  // Make sure the unrelated task really is occupying a global-pool thread before timing anything,
  // so a pass cannot come from the task not having started yet.
  const auto start_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!parked_started.load() && std::chrono::steady_clock::now() < start_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(parked_started.load()) << "the unrelated global-pool task never started";

  // Release from a watchdog rather than only after the timing block below. Against a destructor
  // that waits on the global pool, that destructor blocks on this very task while this thread is
  // the only one that could release it -- neither ever finishes, and the suite hangs instead of
  // reporting. A bounded release turns that into a clean assertion failure.
  std::thread watchdog([&release]() {
      for (int i = 0; i < 300 && !release.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      release.store(true);
    });

  const auto begin = std::chrono::steady_clock::now();
  {
    auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
    ASSERT_TRUE(sm->activate());
    sm->deactivate();
  }
  const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - begin);

  EXPECT_LT(elapsed.count(), 1000)
    << "destroying a state machine took " << elapsed.count() << " ms while an unrelated task sat "
       "on the global QtConcurrent pool -- teardown is waiting for work this machine does not own";

  release.store(true);
  watchdog.join();
  // Do not leave the parked task running into the next test.
  QThreadPool::globalInstance()->waitForDone(-1);
}
