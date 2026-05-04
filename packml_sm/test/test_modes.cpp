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
// Mode-switching tests.  PackML modes are integer-typed (ModeType) and
// configured via `packml_sm_generate_modes` which produces compile-time
// constants.  Each mode carries an AvailableStates mask determining which
// PackML states the machine may transition into.  The generic interface
// supports OEM-defined modes beyond the canonical Production / Maintenance /
// Manual.
// ---

#include <chrono>
#include <gtest/gtest.h>

// default_modes.hpp specialises packml_sm::to_string<ModeType>; it must be
// included before any header that triggers instantiation of the generic
// template (e.g. state_machine.hpp).
#include "default_modes.hpp"  // generated from modes/default_modes.yaml

#include "packml_sm/common.hpp"
#include "packml_sm/state_machine.hpp"
#include "test_helpers.hpp"

using packml_sm::AvailableStates;
using packml_sm::State;
using packml_sm_test::wait_for_state;
using packml_sm_test::execute_success_long;
using packml_sm_test::drive_to_idle;

namespace
{
auto fresh_sm()
{
  auto sm = packml_sm::StateMachine::continuousCycleSM(packml_sm_test::kTestDelayMs);
  sm->setExecute(std::bind(execute_success_long));
  sm->activate();
  return sm;
}

// All states available — equivalent to default Production behaviour.
AvailableStates fully_open()
{
  return {
    {State::ABORTING, true}, {State::ABORTED, true},
    {State::CLEARING, true}, {State::STOPPING, true},
    {State::STOPPED, true},  {State::RESETTING, true},
    {State::IDLE, true},     {State::STARTING, true},
    {State::EXECUTE, true},  {State::HOLDING, true},
    {State::HELD, true},     {State::UNHOLDING, true},
    {State::SUSPENDING, true}, {State::SUSPENDED, true},
    {State::UNSUSPENDING, true}, {State::COMPLETING, true},
    {State::COMPLETE, true},
  };
}
}  // namespace

TEST(Modes, ChangeModeFromUninitializedSucceeds)
{
  auto sm = fresh_sm();
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  // Cold-start: currentMode.name is empty -> mode switch allowed even outside IDLE.
  auto rc = sm->changeMode(packml_modes::Production);
  EXPECT_TRUE(rc.has_value()) << (rc.has_value() ? "" : rc.error());
  sm->deactivate();
}

TEST(Modes, ChangeModeOutsideIdleIsRejectedAfterInit)
{
  auto sm = fresh_sm();
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  // First switch establishes currentMode.
  ASSERT_TRUE(sm->changeMode(packml_modes::Production).has_value());
  // Still in STOPPED — second switch must be rejected.
  auto rc = sm->changeMode(packml_modes::Maintenance);
  EXPECT_FALSE(rc.has_value())
    << "PackML: mode switch only permitted from IDLE (per StatesGenerator::switch_states).";
  sm->deactivate();
}

TEST(Modes, ChangeModeFromIdleSucceeds)
{
  auto sm = fresh_sm();
  ASSERT_TRUE(drive_to_idle(*sm));
  auto rc = sm->changeMode(packml_modes::Maintenance);
  EXPECT_TRUE(rc.has_value()) << (rc.has_value() ? "" : rc.error());
  sm->deactivate();
}

// Per-mode AvailableStates mask: HOLDING/HELD/UNHOLDING disabled => HOLD
// command from EXECUTE must be rejected (transition's target is unavailable).
TEST(Modes, MaskDisablesTransitionToUnavailableState)
{
  auto sm = fresh_sm();
  ASSERT_TRUE(drive_to_idle(*sm));

  AvailableStates restricted = fully_open();
  restricted[State::HOLDING]   = false;
  restricted[State::HELD]      = false;
  restricted[State::UNHOLDING] = false;

  auto rc = sm->changeMode(packml_modes::Production, restricted);
  ASSERT_TRUE(rc.has_value()) << (rc.has_value() ? "" : rc.error());

  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));

  // HOLD should be rejected because target HOLDING is masked-off.
  EXPECT_FALSE(sm->hold())
    << "When HOLDING is masked unavailable, HOLD must be rejected.";
  EXPECT_EQ(sm->getCurrentState(), State::EXECUTE);
  sm->deactivate();
}

TEST(Modes, MaskAllowingHoldKeepsHoldAccepted)
{
  auto sm = fresh_sm();
  ASSERT_TRUE(drive_to_idle(*sm));
  ASSERT_TRUE(sm->changeMode(packml_modes::Production, fully_open()).has_value());
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  EXPECT_TRUE(sm->hold());
  EXPECT_TRUE(wait_for_state(*sm, State::HELD, std::chrono::seconds(5)));
  sm->deactivate();
}

TEST(Modes, OEMDefinedModeIsAcceptedByGenericInterface)
{
  auto sm = fresh_sm();
  ASSERT_TRUE(drive_to_idle(*sm));
  // Any int is a valid ModeType -- modes are open-ended per PackML.
  packml_sm::ModeType custom = 42;
  auto rc = sm->changeMode(custom);
  EXPECT_TRUE(rc.has_value()) << (rc.has_value() ? "" : rc.error());
  sm->deactivate();
}
