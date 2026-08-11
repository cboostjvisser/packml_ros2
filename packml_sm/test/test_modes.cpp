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

#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <thread>

#include "packml_sm/common.hpp"
#include "packml_sm/state_machine.hpp"
#include "packml_sm/default_modes.hpp"
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
  EXPECT_TRUE(sm->activate());
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

// ---------------------------------------------------------------------------
// Mandatory states. A mode may not disable STOPPED, IDLE, EXECUTE, ABORTED or ABORTING, and
// enabling an acting state makes its paired wait state mandatory too (HOLDING/UNHOLDING -> HELD,
// SUSPENDING/UNSUSPENDING -> SUSPENDED, COMPLETING -> COMPLETE). A mask that breaks a rule is
// REPAIRED on the way in rather than rejected -- see enforce_mandatory_states() in common.hpp.
//
// Those rules are what make an acting state with an unreachable target impossible to configure:
// every acting state's own target is either always mandatory or made mandatory by the acting
// state being enabled. The two tests below feed masks that break a rule and check the repair,
// then check the behaviour that repair protects.
//
// Keeping the mask off StateCompleteTransition and ErrorTransition is a second line of defence
// behind that, not the primary rule -- see packml_transitions.hpp.
// ---------------------------------------------------------------------------

// EXECUTE is mandatory, so a mask disabling it is repaired rather than applied. Without that
// repair START would be admitted (its own target, STARTING, is available) and the machine would
// enter STARTING, whose only exit is its state-complete transition to a state the mask forbids --
// stranded with no exit, no alarm and no timeout, while START had already reported success.
// Every mandatory-state rule, checked directly on the repair function rather than through a
// running machine: the four unconditional states plus ABORTING, and the three pairs that become
// mandatory only because their acting state is enabled.
TEST(Modes, EnforceMandatoryStatesRestoresWhatAModeMayNotDisable)
{
  AvailableStates all_off;
  for (const auto & [state, available] : fully_open()) {
    all_off[state] = false;
  }
  auto restored = packml_sm::enforce_mandatory_states(all_off);
  for (const auto state : {State::STOPPED, State::IDLE, State::EXECUTE, State::ABORTED,
      State::ABORTING})
  {
    EXPECT_TRUE(all_off.at(state)) << to_string(state) << " is mandatory and was not restored";
  }
  EXPECT_FALSE(restored.empty()) << "repairs were made but not reported to the caller";

  // With every acting state off, no pair is triggered, so the wait states stay maskable.
  EXPECT_FALSE(all_off.at(State::HELD));
  EXPECT_FALSE(all_off.at(State::SUSPENDED));
  EXPECT_FALSE(all_off.at(State::COMPLETE));

  // Enabling one half of a pair makes its wait state mandatory.
  AvailableStates unholding_only = all_off;
  unholding_only[State::UNHOLDING] = true;
  packml_sm::enforce_mandatory_states(unholding_only);
  EXPECT_TRUE(unholding_only.at(State::HELD))
    << "UNHOLDING is enabled, so HELD must be restored";

  AvailableStates completing_only = all_off;
  completing_only[State::COMPLETING] = true;
  packml_sm::enforce_mandatory_states(completing_only);
  EXPECT_TRUE(completing_only.at(State::COMPLETE))
    << "COMPLETING is enabled, so COMPLETE must be restored";

  // A mask that breaks nothing is returned untouched.
  AvailableStates open = fully_open();
  EXPECT_TRUE(packml_sm::enforce_mandatory_states(open).empty());
  EXPECT_EQ(open, fully_open());
}

TEST(Modes, MaskDisablingExecuteIsRepairedSoStartingIsNotStranded)
{
  auto sm = fresh_sm();
  ASSERT_TRUE(drive_to_idle(*sm));

  AvailableStates execute_masked = fully_open();
  execute_masked[State::EXECUTE] = false;
  ASSERT_TRUE(sm->changeMode(packml_modes::Production, execute_masked).has_value());

  EXPECT_TRUE(sm->getAvailableStates().at(State::EXECUTE))
    << "EXECUTE is mandatory and must have been restored";

  ASSERT_TRUE(sm->start()) << "START targets STARTING, which is available, so it must be admitted";
  EXPECT_TRUE(wait_for_state(*sm, State::EXECUTE, std::chrono::seconds(5)))
    << "machine did not complete STARTING -> EXECUTE; it is stranded in state "
    << static_cast<int>(sm->getCurrentState());
  sm->deactivate();
}

// ABORTING is mandatory because Abort must be accepted from any state, so a mask disabling it is
// repaired. That is a safety property: a fault has to be able to escalate regardless of
// configuration, or a mode file decides whether the machine may react to a fault at all.
TEST(Modes, MaskDisablingAbortingIsRepairedSoErrorsStillEscalate)
{
  auto sm = fresh_sm();
  sm->setResetting(std::bind(packml_sm_test::execute_fail));
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));

  AvailableStates aborting_masked = fully_open();
  aborting_masked[State::ABORTING] = false;
  ASSERT_TRUE(sm->changeMode(packml_modes::Production, aborting_masked).has_value());

  EXPECT_TRUE(sm->getAvailableStates().at(State::ABORTING))
    << "ABORTING is mandatory and must have been restored";

  ASSERT_TRUE(sm->reset()) << "RESET targets RESETTING, which is available";
  EXPECT_TRUE(wait_for_state(*sm, State::ABORTED, std::chrono::seconds(5)))
    << "a failing RESETTING did not escalate -- current state "
    << static_cast<int>(sm->getCurrentState())
    << ". Error escalation must never be mode-gated.";
  sm->deactivate();
}

// A mode's mask is one availability flag per state (PackmlState::availableInMode()), written by
// mode_switcher() on whichever thread calls changeMode and read by CmdTransition on the state
// machine's own thread. Without mode_mask_mutex() the write races the read, and because the flags
// are written one at a time a command can be evaluated against a half-applied mode.
//
// The two masks below differ deliberately, and that is the whole point of this test's shape:
// flipping between two ALL-OPEN masks writes the same value every state already has, never
// mutating the mask and exercising neither half of the defect. Masks that genuinely differ are
// what make the churn real.
//
// SCOPE: this does not PROVE the race is closed. A data race has no defined symptom to assert, and
// catching a half-applied mask means winning the race the lock removes. What it does cover:
// nothing else drives mode changes and command evaluation concurrently, and a lock in the wrong
// place -- a deadlock, or an unresponsive machine -- would surface here immediately.
//
// The machine is held in IDLE deliberately: mode changes are only accepted there, and UNHOLD is
// invalid there, so both threads stay hot -- every mode change writes the mask and every command
// reads it.
TEST(Modes, ConcurrentModeChangesAndCommandEvaluationStaySafe)
{
  auto sm = fresh_sm();
  ASSERT_TRUE(drive_to_idle(*sm));

  AvailableStates hold_masked = fully_open();
  hold_masked[State::HOLDING] = false;
  hold_masked[State::HELD] = false;
  const AvailableStates open = fully_open();

  std::atomic<bool> stop{false};
  std::atomic<int> mode_changes{0};
  std::thread mode_thread([&sm, &stop, &mode_changes, &hold_masked, &open]() {
      while (!stop.load()) {
        sm->changeMode(packml_modes::Maintenance, hold_masked);
        sm->changeMode(packml_modes::Production, open);
        mode_changes.fetch_add(2);
      }
    });

  for (int i = 0; i < 200; ++i) {
    sm->changeState(packml_sm::TransitionCmd::UNHOLD);  // invalid from IDLE: evaluated, refused
  }

  stop.store(true);
  mode_thread.join();

  EXPECT_GT(mode_changes.load(), 0) << "the mode thread never got a change through";
  EXPECT_EQ(sm->getCurrentState(), State::IDLE)
    << "the machine left IDLE during the concurrent mode/command churn -- current state "
    << static_cast<int>(sm->getCurrentState());

  // Still responsive, and the last mask applied is really in effect rather than half of one.
  ASSERT_TRUE(sm->changeMode(packml_modes::Production, open).has_value());
  EXPECT_TRUE(sm->start())
    << "START was refused after the churn, so the mask was left in a state nobody selected";
  EXPECT_TRUE(wait_for_state(*sm, State::EXECUTE));
  sm->deactivate();
}

// A mask names states by enum, and the generator looks each one up in a map of raw pointers keyed
// by name. A key the machine does not have -- UNDEFINED is the reachable one, since it is a real
// State value that no generated state carries -- inserts a null into that map on lookup and is
// then written through. It is also the first key a std::map<State, bool> iterates, so an unguarded
// switch dies on it before reaching anything else.
//
// Refusing the switch whole is what makes this safe rather than merely non-crashing: the mask is
// resolved completely before any flag is written, so a mask that names an unknown state leaves the
// previous mode entirely in place instead of a partly-overwritten mixture of the two.
TEST(Modes, MaskNamingAnUnknownStateIsRefusedAndLeavesThePreviousModeIntact)
{
  auto sm = fresh_sm();
  ASSERT_TRUE(drive_to_idle(*sm));

  AvailableStates hold_masked = fully_open();
  hold_masked[State::HOLDING] = false;
  hold_masked[State::HELD] = false;
  ASSERT_TRUE(sm->changeMode(packml_modes::Maintenance, hold_masked).has_value());
  // UNHOLDING is still open, which makes HELD mandatory -- the machine applies the repaired mask,
  // so that is what the assertions below must expect.
  packml_sm::enforce_mandatory_states(hold_masked);

  AvailableStates names_unknown_state = fully_open();
  names_unknown_state[State::UNDEFINED] = false;

  auto rc = sm->changeMode(packml_modes::Production, names_unknown_state);
  ASSERT_FALSE(rc.has_value())
    << "a mask naming a state the machine does not have was accepted";
  EXPECT_NE(rc.error().find("UNDEFINED"), std::string::npos)
    << "the refusal does not say which state was unknown: " << rc.error();

  EXPECT_EQ(sm->getCurrentMode(), packml_modes::Maintenance)
    << "a refused mode switch changed the reported mode";
  EXPECT_EQ(sm->getAvailableStates(), hold_masked)
    << "a refused mode switch left part of its mask behind";

  // The behavioural half: the refused mask was all-open, so if any of it had landed HOLD would be
  // admitted from EXECUTE.
  ASSERT_TRUE(sm->start());
  ASSERT_TRUE(wait_for_state(*sm, State::EXECUTE));
  EXPECT_FALSE(sm->hold())
    << "HOLD was admitted, so the refused all-open mask reached the states after all";
  sm->deactivate();
}

// getCurrentMode() and getAvailableStates() report the mode and the mask that selected it. Both
// read the single pair the generator commits under mode_mask_mutex(); a second copy on
// StateMachine, assigned after mode_switcher() released the lock, would let a reader catch the two
// disagreeing with the flags actually in force -- and, since AvailableStates is a std::map, catch
// that copy mid-assignment.
//
// SCOPE: neither the disagreement window nor the torn map read has a symptom a test can assert.
// This pins the reporting contract, not the storage behind it.
TEST(Modes, ReportedModeAndMaskAreTheOnesActuallyApplied)
{
  auto sm = fresh_sm();
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));

  // Before any mode is applied. Checked here rather than after drive_to_idle(), which installs
  // an all-open mask of its own to get the machine moving.
  EXPECT_EQ(sm->getCurrentMode(), packml_modes::Invalid)
    << "a machine that has never had a mode applied reports one";
  EXPECT_TRUE(sm->getAvailableStates().empty())
    << "a machine that has never had a mode applied reports a mask";

  ASSERT_TRUE(drive_to_idle(*sm));

  // SUSPENDING rather than EXECUTE: EXECUTE is mandatory, so a mask disabling it is repaired on
  // the way in and could not show a difference between what was handed in and what was applied.
  AvailableStates suspend_masked = fully_open();
  suspend_masked[State::SUSPENDING] = false;
  suspend_masked[State::SUSPENDED] = false;
  suspend_masked[State::UNSUSPENDING] = false;
  ASSERT_TRUE(sm->changeMode(packml_modes::Manual, suspend_masked).has_value());

  EXPECT_EQ(sm->getCurrentMode(), packml_modes::Manual);
  EXPECT_EQ(sm->getAvailableStates(), suspend_masked);

  // A switch refused for the ordinary reason -- not from IDLE -- must not move either half.
  ASSERT_TRUE(sm->stop());
  ASSERT_TRUE(wait_for_state(*sm, State::STOPPED));
  ASSERT_FALSE(sm->changeMode(packml_modes::Production, fully_open()).has_value());
  EXPECT_EQ(sm->getCurrentMode(), packml_modes::Manual)
    << "a mode switch refused outside IDLE still updated the reported mode";
  EXPECT_EQ(sm->getAvailableStates(), suspend_masked)
    << "a mode switch refused outside IDLE still updated the reported mask";
  sm->deactivate();
}
