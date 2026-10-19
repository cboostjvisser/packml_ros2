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
// SHAKEDOWN suite (separate binary from packml_ros_tests), TransitionGuard pure-logic group.
//
// Unlike this suite's other probe files, these need no manager, no Equipment Module and no ROS
// graph at all: TransitionGuard is plain, mutex-guarded logic, so the exact call ORDER that a
// race produces can be replayed deterministically and instantly, with zero timing sensitivity.
// That is the point -- the sequence below is reachable in production only when a genuine race
// fires, which makes it nearly unobservable in an integration test, but it is trivial to state
// exactly once written as a call sequence.

#include <gtest/gtest.h>

#include "packml_ros/transition_guard.hpp"
#include "packml_sm/common.hpp"

namespace {

constexpr auto kMode = static_cast<packml_sm::ModeType>(0);

}  // namespace

// ============================================================================
// Reverse-order race: the status echo for a goal arrives before that goal's own
// begin_transition(). Admission-time snapshotting and a change-GATED clearing of
// waiting_for_state_ are each correct alone; together they would strand the guard.
//
// The sequence, exactly as the reverse-order race produces it:
//   1. A goal for RESETTING is admitted     -> note_goal_admitted(RESETTING) snapshots
//                                              current_state_ (STOPPED) at admission.
//   2. The manager's status echo for that SAME goal arrives FIRST
//                                           -> on_status_update(RESETTING) sets
//                                              current_state_ = RESETTING.
//   3. begin_transition() finally runs      -> request_state(RESETTING) correctly REFUSES the
//                                              already_there shortcut (racing_echo_matched_this_goal),
//                                              so the node really does its work. It therefore
//                                              falls through and ARMS waiting_for_state_.
//   4. The node finishes and adopts locally -> mark_state_locally_reached(RESETTING), which is
//                                              on_status_update(RESETTING, ...). But current_state_
//                                              is ALREADY RESETTING from step 2, so the
//                                              `if (state != current_state_)` block does not run
//                                              and waiting_for_state_ is NEVER CLEARED.
//   5. The next coordinated goal, for a DIFFERENT state, is rejected with "Rejected: a state
//      transition is already in progress" -> the goal is aborted -> the manager's coordinated
//      wait fails -> the machine routes to ABORTING.
//
// A release rule keyed on the state having CHANGED cannot fire at step 4, so it would leave the
// arm set and the next goal for a different state rejected -- aborting that goal, failing the
// manager's coordinated wait, and collapsing a running machine to ABORTING. The arm is therefore
// scope-bound: released by the request that created it on every terminal outcome, never by a
// status echo. This test pins that release.
//
// Step 2 needs the reverse-order race to actually fire, which the manager's bounded ~200ms
// acceptance wait normally suppresses -- so removing that wait with no replacement makes this
// sequence routine rather than rare.
TEST(ShakedownGuardTest, ReverseOrderRaceThenLocalAdoptionReleasesTheArm)
{
  packml_ros::TransitionGuard guard;

  // Baseline: the node believes it is STOPPED.
  guard.on_status_update(packml_sm::State::STOPPED, kMode);

  {
    // 1. A RESETTING goal is admitted. current_state_ is still STOPPED here -- structurally, since
    //    the SendGoal response that lets the manager publish cannot be sent until this returns --
    //    so this is where "am I already there" gets its uncontaminated answer.
    guard.admit_state(packml_sm::State::RESETTING);

    // 2. The status echo for that same goal wins the race and lands first.
    guard.on_status_update(packml_sm::State::RESETTING, kMode);

    // 3. The goal's own begin_transition() runs and takes the answer decided at step 1, so the
    //    racing echo cannot turn this into a no-op and the node genuinely performs the work.
    const auto resetting = guard.claim_state(packml_sm::State::RESETTING);
    EXPECT_TRUE(resetting.result.accepted) << "the racing goal should still be accepted";
    ASSERT_FALSE(resetting.result.already_there)
      << "PRECONDITION FAILED: the admission-time decision did not survive the racing echo, so "
         "this test is no longer replaying the reverse-order race it exists to cover";

    // 4. The node finishes RESETTING and adopts the state locally. current_state_ is already
    //    RESETTING from step 2, so this reports no change at all -- the shape a change-gated
    //    release rule cannot fire on.
    EXPECT_FALSE(guard.on_status_update(packml_sm::State::RESETTING, kMode))
      << "sanity: the local adoption is a no-op change, which is the shape that stalls a "
         "change-gated release";
  }
  // 5. The goal is over, so its arm is too -- released by leaving the scope above rather than by
  //    any status echo.
  const auto idle = guard.request_state(packml_sm::State::IDLE);
  EXPECT_TRUE(idle.result.accepted)
    << "the arm from the completed RESETTING goal was not released, so a legitimate goal for a "
       "different state was rejected -- downstream that aborts the goal, fails the manager's "
       "coordinated wait, and collapses a running machine to ABORTING";
}

// ============================================================================
// The control for the test above: the SAME goal sequence without the racing echo. Proves the
// wedge is specific to the reverse-order ordering, not to ordinary
// request_state -> complete -> request_state cycling (which must keep working, and does).
TEST(ShakedownGuardTest, NormalOrderGoalThenLocalAdoption_GuardClearsAndAcceptsNextGoal)
{
  packml_ros::TransitionGuard guard;
  guard.on_status_update(packml_sm::State::STOPPED, kMode);

  {
    guard.admit_state(packml_sm::State::RESETTING);

    // No echo this time -- the goal's begin_transition() runs first, as intended.
    const auto resetting = guard.claim_state(packml_sm::State::RESETTING);
    ASSERT_TRUE(resetting.result.accepted);
    ASSERT_FALSE(resetting.result.already_there);

    // Local adoption DOES change current_state_ here. It adopts the state; it no longer has any
    // say over the arm, which ends with the request below.
    EXPECT_TRUE(guard.on_status_update(packml_sm::State::RESETTING, kMode));
  }

  const auto idle = guard.request_state(packml_sm::State::IDLE);
  EXPECT_TRUE(idle.result.accepted)
    << "ordinary (non-racing) cycling regressed, which would be a far broader problem: "
    << idle.result.error;
}

// ============================================================================
// Mode-arm release: a release must belong to the request that armed it.
//
// SCOPE: no sequence through the NORMAL manager path strands a mode arm, because request_mode()
// only arms when the target differs from current_mode_, and the only thing that sets current_mode_
// to the target is a status update -- which a change-gated rule already disarms on.
//
// What this test pins is the discriminating case below: while armed for one mode, statuses that
// keep reporting the PREVIOUS mode cannot release the arm under a change-gated rule, and
// request_state() rejects on waiting_for_mode_ UNCONDITIONALLY -- so every state goal on the node
// is refused for as long as that lasts. Whether the manager can emit that sequence is open; the
// guard should not depend on the answer.
TEST(ShakedownGuardTest, ModeArmSurvivesStatusesReportingThePreviousMode)
{
  packml_ros::TransitionGuard guard;
  constexpr auto kModeA = static_cast<packml_sm::ModeType>(1);
  constexpr auto kModeB = static_cast<packml_sm::ModeType>(2);

  guard.on_status_update(packml_sm::State::STOPPED, kModeA);
  ASSERT_TRUE(guard.request_mode(kModeB).accepted);   // arms for B; current_mode_ is A

  // Statuses continue to report A -- the mode this node is NOT waiting for. A change-gated rule
  // could not disarm on any of these, because A == current_mode_ already.
  for (int i = 0; i < 5; ++i) {
    guard.on_status_update(packml_sm::State::STOPPED, kModeA);
  }

  // The arm is still legitimately outstanding here (B was never confirmed), so a state goal
  // is correctly refused.
  EXPECT_FALSE(guard.request_state(packml_sm::State::RESETTING).result.accepted)
    << "an outstanding, unconfirmed mode change should still block a state goal";

  // Now B is confirmed -- the request's own target. This must release the arm, and does so
  // because the release is keyed on switching_mode_ rather than on the mode having changed.
  guard.on_status_update(packml_sm::State::STOPPED, kModeB);
  const auto resetting = guard.request_state(packml_sm::State::RESETTING);
  EXPECT_TRUE(resetting.result.accepted)
    << "the mode arm was not released by confirmation of its own target: " << resetting.result.error;
}

// Repeated and redundant mode confirmations leave no arm stranded.
//
// SCOPE: this does NOT isolate the switching_mode-keyed release, and cannot. That release lives in
// an `else if` reached only when the arriving mode EQUALS current_mode while an arm for that same
// mode is outstanding, and the guard's own API cannot produce that combination: request_mode()
// refuses to arm when the target already equals current_mode (transition_guard.cpp's first check),
// and the only writer of current_mode is on_status_update()'s first branch, which clears the arm on
// its way past. Every arm below is released by the change-gated branch instead, so the keyed
// release is defensive hardening rather than a path this test reaches. It is untested BY
// CONSTRUCTION, not by omission, and transition_guard.cpp states that at the branch itself.
//
// No wedge of that shape has been reproduced, and none is reachable for the reason above. A
// reachable trace would be news.
//
// What this covers is the reachable half: ordinary and repeated mode churn, including a duplicate
// echo, never leaves the guard refusing subsequent state requests.
TEST(ShakedownGuardTest, RepeatedModeConfirmationsLeaveNoArmStranded)
{
  packml_ros::TransitionGuard guard;
  constexpr auto kModeA = static_cast<packml_sm::ModeType>(1);
  constexpr auto kModeB = static_cast<packml_sm::ModeType>(2);

  guard.on_status_update(packml_sm::State::STOPPED, kModeA);
  ASSERT_TRUE(guard.request_mode(kModeB).accepted);
  guard.on_status_update(packml_sm::State::STOPPED, kModeB);   // adopts B, releases
  ASSERT_TRUE(guard.request_mode(kModeA).accepted);            // re-arms for A
  guard.on_status_update(packml_sm::State::STOPPED, kModeA);    // adopts A, releases

  // A second, redundant A -- no change. Must remain released, not re-strand anything.
  guard.on_status_update(packml_sm::State::STOPPED, kModeA);
  EXPECT_FALSE(guard.is_switching_mode())
    << "a redundant mode echo re-armed the guard";
  EXPECT_TRUE(guard.request_state(packml_sm::State::RESETTING).result.accepted);
}
