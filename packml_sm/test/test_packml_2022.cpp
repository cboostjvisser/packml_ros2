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

#include <gtest/gtest.h>
#include <stop_token>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>

#include "packml_sm/modes_config.hpp"
#include "packml_sm/state_machine.hpp"
#include "test_helpers.hpp"

using packml_sm::State;
using packml_sm::TransitionCmd;
using packml_sm_test::wait_for_state;
using namespace std::chrono_literals;

namespace
{
struct Entries
{
  std::atomic<int> execute{0};
  std::atomic<int> completing{0};
  std::atomic<int> completed{0};
  std::atomic<bool> completed_name{false};
};

std::shared_ptr<Entries> observe(const std::shared_ptr<packml_sm::StateMachine> & sm)
{
  auto entries = std::make_shared<Entries>();
  sm->on_state_changed = [entries](State state, const QString & name) {
      if (state == State::EXECUTE) {
        ++entries->execute;
      }
      if (state == State::COMPLETING) {
        ++entries->completing;
      }
      if (state == State::COMPLETED) {
        entries->completed_name.store(name == "COMPLETED");
        ++entries->completed;
      }
    };
  return entries;
}

bool enter_production_state(packml_sm::StateMachine & sm, State target)
{
  if (!sm.start() || !wait_for_state(sm, State::EXECUTE)) {
    return false;
  }
  if (target == State::HELD) {
    return sm.hold() && wait_for_state(sm, State::HELD);
  }
  if (target == State::SUSPENDED) {
    return sm.suspend() && wait_for_state(sm, State::SUSPENDED);
  }
  return target == State::EXECUTE;
}

// Both factory APIs expose the same 2022 graph. Only their unbound Execute behavior differs.
class Packml2022 : public ::testing::TestWithParam<bool>
{
protected:
  std::shared_ptr<packml_sm::StateMachine> make_machine()
  {
    if (GetParam()) {
      return packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
    }
    return packml_sm::StateMachine::singleCycleSM(packml_sm_test::kTestDelayMs);
  }
};
}  // namespace

TEST_P(Packml2022, CompleteEndsExecuteHeldAndSuspendedThroughCompleting)
{
  for (const auto source : {State::EXECUTE, State::HELD, State::SUSPENDED}) {
    SCOPED_TRACE(packml_sm::to_string(source));
    auto sm = make_machine();
    auto entries = observe(sm);
    auto interrupted = std::make_shared<std::atomic<bool>>(false);
    ASSERT_TRUE(sm->setInterruptibleStateOperation(State::EXECUTE,
      [interrupted](std::stop_token token) {
        packml_sm_test::execute_hold(token);
        interrupted->store(token.stop_requested());
        return 0;
      }));
    ASSERT_TRUE(sm->activate());
    ASSERT_TRUE(packml_sm_test::drive_to_idle(*sm));
    ASSERT_TRUE(enter_production_state(*sm, source));

    // Exercise the command-number dispatch used by external clients, not only complete().
    const auto result = sm->changeState(TransitionCmd::COMPLETE);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result.value());
    ASSERT_TRUE(wait_for_state(*sm, State::COMPLETED));
    // The command acknowledgement and state publication need not join the Execute worker.
    // Deactivation below drains it; first prove the stop request reaches it promptly.
    const auto interrupted_deadline = std::chrono::steady_clock::now() + 3s;
    while (!interrupted->load() && std::chrono::steady_clock::now() < interrupted_deadline) {
      std::this_thread::sleep_for(1ms);
    }
    EXPECT_TRUE(interrupted->load());
    // This rejected command also drains any pending state-entry notification before inspection.
    EXPECT_FALSE(sm->complete()) << "Completed must reject another Complete command";
    EXPECT_EQ(entries->execute.load(), 1);
    EXPECT_EQ(entries->completing.load(), 1);
    EXPECT_EQ(entries->completed.load(), 1);
    EXPECT_TRUE(entries->completed_name.load());

    ASSERT_TRUE(sm->reset());
    EXPECT_TRUE(wait_for_state(*sm, State::IDLE));
    sm->deactivate();
  }
}

TEST_P(Packml2022, SuccessfulExecuteCompletesBatchOnceWithoutReentry)
{
  auto sm = make_machine();
  auto entries = observe(sm);
  auto calls = std::make_shared<std::atomic<int>>(0);
  ASSERT_TRUE(sm->setExecute([calls] {
      ++*calls;
      return 0;
    }));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::COMPLETED));
  sm->deactivate();
  EXPECT_EQ(calls->load(), 1);
  EXPECT_EQ(entries->execute.load(), 1);
  EXPECT_EQ(entries->completing.load(), 1);
  EXPECT_EQ(entries->completed.load(), 1);
}

TEST_P(Packml2022, SuspendedCanHoldThenUnholdIntoExecute)
{
  auto sm = make_machine();
  ASSERT_TRUE(sm->setInterruptibleStateOperation(State::EXECUTE, packml_sm_test::execute_hold));
  auto holding_calls = std::make_shared<std::atomic<int>>(0);
  ASSERT_TRUE(sm->setStateOperation(State::HOLDING, [holding_calls] {
      ++*holding_calls;
      return 0;
    }));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::drive_to_idle(*sm));
  ASSERT_TRUE(enter_production_state(*sm, State::SUSPENDED));
  ASSERT_TRUE(sm->hold());
  ASSERT_TRUE(wait_for_state(*sm, State::HELD));
  EXPECT_EQ(holding_calls->load(), 1);
  ASSERT_TRUE(sm->unhold());
  EXPECT_TRUE(wait_for_state(*sm, State::EXECUTE));
  ASSERT_TRUE(sm->complete());
  EXPECT_TRUE(wait_for_state(*sm, State::COMPLETED));
  sm->deactivate();
}

TEST_P(Packml2022, CompleteHonorsCompletingModeMaskInEveryProductionState)
{
  for (const auto source : {State::EXECUTE, State::HELD, State::SUSPENDED}) {
    SCOPED_TRACE(packml_sm::to_string(source));
    auto sm = make_machine();
    ASSERT_TRUE(sm->setInterruptibleStateOperation(State::EXECUTE, packml_sm_test::execute_hold));
    ASSERT_TRUE(sm->activate());
    ASSERT_TRUE(packml_sm_test::install_mode_then_idle(
      *sm, 1, {{State::COMPLETING, false}, {State::COMPLETED, false}}));
    ASSERT_TRUE(enter_production_state(*sm, source));
    EXPECT_FALSE(sm->complete());
    EXPECT_EQ(sm->getCurrentState(), source);
    sm->deactivate();
  }
}

TEST_P(Packml2022, HoldFromSuspendedHonorsHoldingModeMask)
{
  auto sm = make_machine();
  ASSERT_TRUE(sm->setInterruptibleStateOperation(State::EXECUTE, packml_sm_test::execute_hold));
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::install_mode_then_idle(*sm, 1, {{State::HOLDING, false}}));
  ASSERT_TRUE(enter_production_state(*sm, State::SUSPENDED));
  EXPECT_FALSE(sm->hold());
  EXPECT_EQ(sm->getCurrentState(), State::SUSPENDED);
  sm->deactivate();
}

INSTANTIATE_TEST_SUITE_P(CycleFactories, Packml2022, ::testing::Bool());

TEST(Packml2022Compatibility, UnboundContinuousExecuteStillHoldsUntilComplete)
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  auto entries = observe(sm);
  ASSERT_TRUE(sm->activate());
  ASSERT_TRUE(packml_sm_test::drive_to_idle(*sm));
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  // Longer than many default-operation delays: no placeholder may fabricate completion.
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(sm->getCurrentState(), State::EXECUTE);
  EXPECT_EQ(entries->execute.load(), 1);
  EXPECT_EQ(entries->completing.load(), 0);
  ASSERT_TRUE(sm->complete());
  EXPECT_TRUE(wait_for_state(*sm, State::COMPLETED));
  sm->deactivate();
}

TEST(Packml2022Compatibility, WireNumbersAndLegacyCompletedAliasStayStable)
{
  EXPECT_EQ(static_cast<int>(TransitionCmd::COMPLETE), 10);
  EXPECT_EQ(packml_sm::to_string(TransitionCmd::COMPLETE), "COMPLETE");
  EXPECT_EQ(static_cast<int>(State::COMPLETED), 17);
  EXPECT_EQ(State::COMPLETE, State::COMPLETED);
  EXPECT_EQ(packml_sm::to_string(State::COMPLETE), "COMPLETED");
}

TEST(Packml2022Compatibility, ModesAcceptCompletedAndLegacyCompleteSpelling)
{
  const auto path = std::filesystem::temp_directory_path() /
    ("packml_2022_modes_" + std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  {
    std::ofstream config(path);
    ASSERT_TRUE(config.is_open());
    config << "modes:\n  Production: 1\n  Maintenance: 2\nstate_masks:\n"
      "  Production:\n    COMPLETING: false\n    COMPLETED: false\n"
      "  Maintenance:\n    COMPLETING: false\n    COMPLETE: false\n";
  }
  const auto masks = packml_sm::parse_modes_config(path.string());
  std::filesystem::remove(path);
  ASSERT_EQ(masks.size(), 2u);
  EXPECT_FALSE(masks.at(1).at(State::COMPLETED));
  EXPECT_FALSE(masks.at(2).at(State::COMPLETED));
}
