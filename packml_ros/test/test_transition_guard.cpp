// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---
// Tests for TransitionGuard and PackmlNodeProtocol - the shared protocol logic
// (rclcpp + rclpy via pybind11) for state/mode transition coordination.

#include <gtest/gtest.h>
#include "packml_ros/transition_guard.hpp"
#include "packml_sm/default_modes.hpp"

using packml_ros::TransitionGuard;
using packml_ros::TransitionResult;
using packml_sm::State;
using packml_sm::ModeType;

class TransitionGuardTest : public ::testing::Test
{
protected:
  TransitionGuard guard;
};

// --- Initial state ---

// A freshly constructed guard starts in UNDEFINED state.
TEST_F(TransitionGuardTest, InitialStateIsUndefined)
{
  EXPECT_EQ(guard.current_state(), State::UNDEFINED);
}

// A freshly constructed guard starts in mode 0 (Invalid).
TEST_F(TransitionGuardTest, InitialModeIsZero)
{
  EXPECT_EQ(guard.current_mode(), packml_modes::Invalid);
}

// A fresh guard reports no state or mode transition in flight.
TEST_F(TransitionGuardTest, NotSwitchingInitially)
{
  EXPECT_FALSE(guard.is_switching_state());
  EXPECT_FALSE(guard.is_switching_mode());
}

// --- State transition requests ---

// A first state request (target != current) is accepted, is not a no-op, and carries no warning.
TEST_F(TransitionGuardTest, AcceptStateTransition)
{
  auto result = guard.request_state(State::STOPPED);
  EXPECT_TRUE(result.result.accepted);
  EXPECT_FALSE(result.result.already_there);
  EXPECT_TRUE(result.result.error.empty());
}

// Requesting the state the guard is already in is a no-op success: accepted + already_there, no warning.
TEST_F(TransitionGuardTest, AlreadyInState)
{
  // Force state to IDLE via status update
  guard.on_status_update(State::IDLE, 0);

  auto result = guard.request_state(State::IDLE);
  EXPECT_TRUE(result.result.already_there);
  EXPECT_TRUE(result.result.accepted);  // also "accepted" (no-op success)
  EXPECT_TRUE(result.result.error.empty());
}

// --- note_goal_admitted() / the reverse-order race ---

// Reproduces the race directly, without any ROS/network timing involved: a status echo for a
// genuinely NEW target reaches on_status_update() before its paired action goal executes. If the
// already_there question were asked at execution it would answer "yes" here and the caller would
// skip on_state_trans_req()/defers_completion() for a transition that was never processed.
// admit_state() asks it at admission instead, where the echo has not landed yet, and claim_state()
// returns that stored answer rather than asking again.
TEST_F(TransitionGuardTest, AdmittedGoalNotShortcutByRacingStatusEcho)
{
  guard.admit_state(State::IDLE);          // goal admission: always happens first, structurally
  guard.on_status_update(State::IDLE, 0);  // the race: status echo for the SAME target wins

  auto claimed = guard.claim_state(State::IDLE);  // begin_transition() finally runs
  EXPECT_TRUE(claimed.result.accepted);
  EXPECT_FALSE(claimed.result.already_there)
    << "a racing status echo for this goal's own target must not make it look like a no-op";
  EXPECT_TRUE(claimed.arm.armed()) << "real work was accepted, so the guard must be armed for it";
}

// An admission for a DIFFERENT target must not suppress the already_there shortcut for an actual
// no-op request.
TEST_F(TransitionGuardTest, UnrelatedAdmissionDoesNotAffectDifferentTargetAlreadyThere)
{
  guard.admit_state(State::EXECUTE);       // admitted goal is for a different state
  guard.on_status_update(State::IDLE, 0);  // node is genuinely, unrelatedly already at IDLE

  auto claimed = guard.claim_state(State::IDLE);
  EXPECT_TRUE(claimed.result.already_there)
    << "an admission for a different target must not affect this unrelated already_there";
}

// An admission is consumed by exactly the one claim for its target. A later, genuinely redundant
// request for that same state must get the ordinary already_there answer.
TEST_F(TransitionGuardTest, AdmissionConsumedOnFirstMatchingClaim)
{
  guard.admit_state(State::IDLE);
  guard.on_status_update(State::IDLE, 0);
  guard.claim_state(State::IDLE);  // consumes the admission, and releases its arm at scope exit

  auto claimed = guard.claim_state(State::IDLE);  // a later, genuinely redundant request
  EXPECT_TRUE(claimed.result.already_there)
    << "the admission must not still be exempting requests after its own goal was processed";
}

// The arm belongs to the request that took it and ends with it, on every outcome -- including the
// ones where the machine never moves. Before this was scope-bound the arm was released only by a
// status reporting a DIFFERENT state, so a request that ended without moving anything, or a node
// adopting exactly what it was asked for, left the guard armed and refused the next goal.
TEST_F(TransitionGuardTest, ArmIsReleasedWhenTheRequestThatTookItEnds)
{
  guard.on_status_update(State::STOPPED, 0);
  {
    auto claimed = guard.request_state(State::RESETTING);
    ASSERT_TRUE(claimed.result.accepted);
    EXPECT_TRUE(guard.is_switching_state());
  }
  EXPECT_FALSE(guard.is_switching_state())
    << "the arm outlived the request that took it";
  EXPECT_TRUE(guard.request_state(State::IDLE).result.accepted)
    << "a stranded arm would reject this";
}

// The specific shape that strands a change-gated release: the node adopts exactly the state it was
// asked for, so on_status_update() sees no change at all and has nothing to clear on.
TEST_F(TransitionGuardTest, LocalAdoptionOfTheRequestedStateDoesNotStrandTheArm)
{
  guard.on_status_update(State::STOPPED, 0);
  {
    auto claimed = guard.request_state(State::RESETTING);
    ASSERT_TRUE(claimed.result.accepted);
    guard.on_status_update(State::RESETTING, 0);  // the echo for this same request
    EXPECT_FALSE(guard.on_status_update(State::RESETTING, 0))
      << "sanity: local adoption is a no-op change, the shape that strands a change-gated release";
  }
  EXPECT_TRUE(guard.request_state(State::IDLE).result.accepted)
    << "the next, different goal must be accepted once this request has ended";
}

// Accepting a state request marks the guard as switching state, for as long as the caller holds
// the arm. A discarded ArmedTransition is a request that already ended, so it leaves nothing
// armed -- which is the whole point of tying the arm to a scope.
TEST_F(TransitionGuardTest, SwitchingStateFlag)
{
  auto claimed = guard.request_state(State::STOPPED);
  EXPECT_TRUE(guard.is_switching_state());
}

// A status CONFIRMING the arm's own target releases it: once the manager has published the state
// this node was switching to, the transition is settled and the node must be free for the next
// goal -- above all an ABORT, which has to be able to interrupt a deferred completion that may
// still legitimately be waiting.
TEST_F(TransitionGuardTest, StatusConfirmingTheArmsOwnTargetReleasesIt)
{
  auto claimed = guard.request_state(State::STOPPED);
  EXPECT_TRUE(guard.is_switching_state());

  guard.on_status_update(State::STOPPED, 0);
  EXPECT_FALSE(guard.is_switching_state())
    << "the manager confirmed the target, so this node must be able to take the next goal";
  EXPECT_EQ(guard.current_state(), State::STOPPED);
}

// A status for some OTHER state must not end an arm that does not belong to it. A rule that
// releases on any change at all lets an unrelated echo free a transition still in flight.
TEST_F(TransitionGuardTest, StatusForAnotherStateDoesNotReleaseTheArm)
{
  auto claimed = guard.request_state(State::RESETTING);
  ASSERT_TRUE(guard.is_switching_state());

  guard.on_status_update(State::EXECUTE, 0);
  EXPECT_TRUE(guard.is_switching_state())
    << "an unrelated status ended an arm it had nothing to do with";
}

// While a state transition is in flight, a request for a DIFFERENT state is rejected
// (accepted=false, with a reason) and the original in-flight target is preserved.
TEST_F(TransitionGuardTest, RejectDifferentStateWhileSwitching)
{
  auto stopped = guard.request_state(State::STOPPED);
  ASSERT_TRUE(stopped.result.accepted);

  // A second, different target must be rejected, not silently override the in-flight STOPPED request.
  auto result = guard.request_state(State::IDLE);
  EXPECT_FALSE(result.result.accepted);
  EXPECT_FALSE(result.result.error.empty());
  EXPECT_FALSE(result.arm.armed()) << "a rejected request must not hold an arm";

  // The original target is preserved and still in flight.
  EXPECT_TRUE(guard.is_switching_state());
  guard.on_status_update(State::STOPPED, packml_modes::Invalid);
  EXPECT_EQ(guard.current_state(), State::STOPPED);
  stopped.arm.release();
  EXPECT_FALSE(guard.is_switching_state());
}

// --- Mode transition requests ---

// A first mode request (target != current) is accepted, is not a no-op, and carries no warning.
TEST_F(TransitionGuardTest, AcceptModeTransition)
{
  auto result = guard.request_mode(packml_modes::Production);
  EXPECT_TRUE(result.accepted);
  EXPECT_FALSE(result.already_there);
  EXPECT_TRUE(result.error.empty());
}

// Requesting the mode the guard is already in is a no-op success: accepted + already_there, no warning.
TEST_F(TransitionGuardTest, AlreadyInMode)
{
  guard.on_status_update(State::UNDEFINED, packml_modes::Maintenance);

  auto result = guard.request_mode(packml_modes::Maintenance);
  EXPECT_TRUE(result.already_there);
  EXPECT_TRUE(result.accepted);
  EXPECT_TRUE(result.error.empty());
}

// Accepting a mode request marks the guard as switching mode.
TEST_F(TransitionGuardTest, SwitchingModeFlag)
{
  guard.request_mode(packml_modes::Production);
  EXPECT_TRUE(guard.is_switching_mode());
}

// A status update to the requested mode clears the switching flag and adopts that mode.
TEST_F(TransitionGuardTest, SwitchingModeClearedByStatusUpdate)
{
  guard.request_mode(packml_modes::Manual);
  EXPECT_TRUE(guard.is_switching_mode());

  guard.on_status_update(State::UNDEFINED, packml_modes::Manual);
  EXPECT_FALSE(guard.is_switching_mode());
  EXPECT_EQ(guard.current_mode(), packml_modes::Manual);
}

// While a mode transition is in flight, a request for a DIFFERENT mode is rejected
// (accepted=false, with a reason) and the original in-flight mode target is preserved.
TEST_F(TransitionGuardTest, RejectDifferentModeWhileSwitching)
{
  guard.request_mode(packml_modes::Production);

  // A second, different mode target must be rejected, not override the in-flight one.
  auto result = guard.request_mode(packml_modes::Maintenance);
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.error.empty());

  // The original mode target is preserved.
  EXPECT_TRUE(guard.is_switching_mode());
  guard.on_status_update(State::UNDEFINED, packml_modes::Production);
  EXPECT_FALSE(guard.is_switching_mode());
  EXPECT_EQ(guard.current_mode(), packml_modes::Production);
}

// --- Cross-interaction: state & mode ---

// At most one transition in flight: a state request is rejected while a mode change
// is still pending, and the guard is NOT armed for the state switch.
TEST_F(TransitionGuardTest, RejectStateWhileModeSwitching)
{
  guard.request_mode(packml_modes::Production);

  // One transition at a time: a state request is rejected while a mode change is in flight.
  auto result = guard.request_state(State::IDLE);
  EXPECT_FALSE(result.result.accepted);
  EXPECT_FALSE(result.result.error.empty());
  EXPECT_FALSE(guard.is_switching_state());  // not armed
}

// Symmetric to the above: a mode request is rejected while a state change is still pending.
TEST_F(TransitionGuardTest, RejectModeWhileStateSwitching)
{
  auto stopped = guard.request_state(State::STOPPED);
  ASSERT_TRUE(stopped.result.accepted);

  auto result = guard.request_mode(1);
  EXPECT_FALSE(result.accepted);
  EXPECT_FALSE(result.error.empty());
  EXPECT_FALSE(guard.is_switching_mode());  // not armed
}

// --- Status updates ---

// A status update adopts the reported state as the guard's current state.
TEST_F(TransitionGuardTest, StatusUpdateChangesState)
{
  guard.on_status_update(State::EXECUTE, 0);
  EXPECT_EQ(guard.current_state(), State::EXECUTE);
}

// A status update adopts the reported mode as the guard's current mode.
TEST_F(TransitionGuardTest, StatusUpdateChangesMode)
{
  guard.on_status_update(State::UNDEFINED, 5);
  EXPECT_EQ(guard.current_mode(), 5);
}

// A status update can adopt a new state and mode in the same call.
TEST_F(TransitionGuardTest, StatusUpdateChangesBoth)
{
  guard.on_status_update(State::IDLE, packml_modes::Maintenance);
  EXPECT_EQ(guard.current_state(), State::IDLE);
  EXPECT_EQ(guard.current_mode(), packml_modes::Maintenance);
}

// on_status_update returns true when the reported state/mode differs from the current one.
TEST_F(TransitionGuardTest, StatusUpdateReturnsTrueOnChange)
{
  EXPECT_TRUE(guard.on_status_update(State::IDLE, 0));
}

// on_status_update returns false when the reported state/mode is unchanged (idempotent no-op).
TEST_F(TransitionGuardTest, StatusUpdateReturnsFalseOnNoChange)
{
  guard.on_status_update(State::IDLE, 1);
  EXPECT_FALSE(guard.on_status_update(State::IDLE, 1));
}
