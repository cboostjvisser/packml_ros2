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
// Pure unit tests for CompletionTracker — no ROS node or executor needed.

#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>

#include "packml_ros/completion_tracker.hpp"

using namespace std::chrono_literals;
using WaitResult = CompletionTracker::WaitResult;

namespace {

rclcpp_action::GoalUUID make_uuid(uint8_t seed)
{
  rclcpp_action::GoalUUID id{};
  id.fill(seed);
  return id;
}

}  // namespace

// A round with no registered nodes completes immediately — an empty set is trivially
// fully-reported.
TEST(CompletionTrackerTest, EmptyRoundCompletesImmediately)
{
  CompletionTracker tracker;
  [[maybe_unused]] const auto round = tracker.begin_round({});
  EXPECT_EQ(tracker.wait_for_all(50ms), WaitResult::COMPLETE);
}

// A node that accepts the goal and then reports success satisfies the wait — even if both
// happen before wait_for_all() is ever called (the predicate is checked on its first pass, no
// actual blocking required).
TEST(CompletionTrackerTest, WaitsUntilNodeReportsSuccess)
{
  CompletionTracker tracker;
  const auto id = make_uuid(1);
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  tracker.on_goal_accepted("em_a", id, round);
  tracker.on_result("em_a", id, true, 0, "", round);
  EXPECT_EQ(tracker.wait_for_all(1s), WaitResult::COMPLETE);
}

// A registered node that never accepts or reports elapses the full timeout and returns TIMEOUT.
TEST(CompletionTrackerTest, TimesOutWhenNoResultArrives)
{
  CompletionTracker tracker;
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  EXPECT_EQ(tracker.wait_for_all(50ms), WaitResult::TIMEOUT);
}

// A result carrying the WRONG (stale) goal id — e.g. a leftover from a previous round — does
// not satisfy the current wait, which times out rather than falsely completing.
TEST(CompletionTrackerTest, StaleGoalIdIgnored)
{
  CompletionTracker tracker;
  const auto old_id = make_uuid(1);
  const auto new_id = make_uuid(2);
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  tracker.on_goal_accepted("em_a", new_id, round);
  tracker.on_result("em_a", old_id, true, 0, "", round);  // a stale/leftover result
  EXPECT_EQ(tracker.wait_for_all(50ms), WaitResult::TIMEOUT);
}

// A result for a node whose goal was never accepted this round (no on_goal_accepted() call
// yet) is discarded — begin_round() alone is not enough to satisfy a node's entry, the
// acceptance must be confirmed first.
TEST(CompletionTrackerTest, ResultWithoutAcceptedGoalIdIgnored)
{
  CompletionTracker tracker;
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  tracker.on_result("em_a", make_uuid(1), true, 0, "", round);
  EXPECT_EQ(tracker.wait_for_all(50ms), WaitResult::TIMEOUT);
}

// An explicit failure result for the current goal id fails the wait FAST — well before the
// full timeout would otherwise elapse.
TEST(CompletionTrackerTest, FailFastOnExplicitFailure)
{
  CompletionTracker tracker;
  const auto id = make_uuid(1);
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  tracker.on_goal_accepted("em_a", id, round);
  tracker.on_result("em_a", id, false, 7, "simulated failure", round);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(tracker.wait_for_all(2s), WaitResult::FAILED);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, 500ms) << "FAILED must be detected on the predicate's first check, "
                               "not after riding out the full timeout";
}

// A goal rejected outright at the ROS admission level also fails the wait fast, the same as
// an explicit failure result (defensive: handle_goal always accepts in this design, but a
// misbehaving/incompatible server should still fail fast, not time out).
TEST(CompletionTrackerTest, GoalRejectionFailsFast)
{
  CompletionTracker tracker;
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  tracker.on_goal_rejected("em_a", "rejected by node", round);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(tracker.wait_for_all(2s), WaitResult::FAILED);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, 500ms);
}

// With two registered nodes, the wait only completes once BOTH have reported success for
// their own accepted goal — one alone is not sufficient.
TEST(CompletionTrackerTest, MultipleNodesAllMustReport)
{
  CompletionTracker tracker;
  const auto id_a = make_uuid(1);
  const auto id_b = make_uuid(2);
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a", "em_b"});
  tracker.on_goal_accepted("em_a", id_a, round);
  tracker.on_goal_accepted("em_b", id_b, round);
  tracker.on_result("em_a", id_a, true, 0, "", round);

  EXPECT_EQ(tracker.wait_for_all(50ms), WaitResult::TIMEOUT)
    << "only one of two nodes has reported";

  tracker.on_result("em_b", id_b, true, 0, "", round);
  EXPECT_EQ(tracker.wait_for_all(1s), WaitResult::COMPLETE);
}

// Starting a NEW round discards all prior-round tracking: a late result tagged with an OLD
// round's goal id must not be mistaken for an answer to the new round, even for the same node
// name.
TEST(CompletionTrackerTest, NewRoundDiscardsStaleTracking)
{
  CompletionTracker tracker;
  const auto old_id = make_uuid(1);
  const auto new_id = make_uuid(2);

  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  tracker.on_goal_accepted("em_a", old_id, round);

  // A new round starts before em_a's result for the old one ever arrived.
  const auto round2 = tracker.begin_round({"em_a"});
  tracker.on_goal_accepted("em_a", new_id, round2);

  // The late result for the OLD goal id must not satisfy the new round.
  tracker.on_result("em_a", old_id, true, 0, "", round2);
  EXPECT_EQ(tracker.wait_for_all(50ms), WaitResult::TIMEOUT);

  // The result for the actual current goal id does satisfy it.
  tracker.on_result("em_a", new_id, true, 0, "", round2);
  EXPECT_EQ(tracker.wait_for_all(1s), WaitResult::COMPLETE);
}

// pending_nodes() reports exactly the nodes with no result yet — used by the manager to know
// which outstanding goals are still worth cancelling after a TIMEOUT.
TEST(CompletionTrackerTest, PendingNodesReflectsUnansweredOnes)
{
  CompletionTracker tracker;
  const auto id_a = make_uuid(1);
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a", "em_b"});
  tracker.on_goal_accepted("em_a", id_a, round);
  tracker.on_result("em_a", id_a, true, 0, "", round);

  const auto pending = tracker.pending_nodes();
  ASSERT_EQ(pending.size(), 1u);
  EXPECT_EQ(pending[0], "em_b");
}

// A coordinated node found unhealthy (health predicate returns false) aborts the wait
// immediately, well before the full timeout.
TEST(CompletionTrackerTest, HealthCrossCheckAbortsWaitFast)
{
  CompletionTracker tracker([](const std::string &) -> std::optional<bool> {return false;});
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});

  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(tracker.wait_for_all(2s), WaitResult::ABORTED_BY_HEALTH);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, 500ms) << "ABORTED_BY_HEALTH must be detected fast, not after the "
                               "full timeout";
}

// A node the health predicate reports as unregistered (nullopt) is treated as health-blind:
// it must NOT abort the wait, only the completion timeout can end it.
TEST(CompletionTrackerTest, HealthBlindNodeDoesNotAbortWait)
{
  CompletionTracker tracker([](const std::string &) -> std::optional<bool> {return std::nullopt;});
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  EXPECT_EQ(tracker.wait_for_all(50ms), WaitResult::TIMEOUT);
}

// A healthy node's health predicate returning true does not interfere with a normal, real
// completion report.
TEST(CompletionTrackerTest, HealthyNodeCompletesNormally)
{
  CompletionTracker tracker([](const std::string &) -> std::optional<bool> {return true;});
  const auto id = make_uuid(1);
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  tracker.on_goal_accepted("em_a", id, round);
  tracker.on_result("em_a", id, true, 0, "", round);
  EXPECT_EQ(tracker.wait_for_all(1s), WaitResult::COMPLETE);
}

// request_shutdown() called from another thread wakes an in-flight wait_for_all()
// immediately, well before its (long) timeout would otherwise elapse.
TEST(CompletionTrackerTest, ShutdownAbortsInFlightWaitFast)
{
  CompletionTracker tracker;
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});

  WaitResult result = WaitResult::COMPLETE;
  const auto start = std::chrono::steady_clock::now();
  std::thread waiter([&]() {
      result = tracker.wait_for_all(5s);
    });
  std::this_thread::sleep_for(50ms);
  tracker.request_shutdown();
  waiter.join();
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_EQ(result, WaitResult::SHUTDOWN);
  EXPECT_LT(elapsed, 1s) << "request_shutdown() must wake an in-flight wait promptly, "
                            "not block process teardown for the full timeout";
}

// notify_health_change() called from another thread, after the health predicate starts
// reporting the node unhealthy, wakes an in-flight wait_for_all() promptly rather than
// waiting for its next natural wakeup.
TEST(CompletionTrackerTest, NotifyHealthChangeWakesWaitPromptly)
{
  std::atomic<bool> healthy{true};
  CompletionTracker tracker([&healthy](const std::string &) -> std::optional<bool> {
      return healthy.load();
    });
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});

  WaitResult result = WaitResult::COMPLETE;
  const auto start = std::chrono::steady_clock::now();
  std::thread waiter([&]() {
      result = tracker.wait_for_all(5s);
    });
  std::this_thread::sleep_for(50ms);
  healthy.store(false);
  tracker.notify_health_change();
  waiter.join();
  const auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_EQ(result, WaitResult::ABORTED_BY_HEALTH);
  EXPECT_LT(elapsed, 1s) << "notify_health_change() must wake an in-flight wait promptly";
}

// ---------------------------------------------------------------------------
// Predicate ordering and the ABORTING opt-out. Both tests below fail if the predicate
// evaluates node health BEFORE a node's own result.
// ---------------------------------------------------------------------------

// A node that has ALREADY reported success must count as done even if it then goes
// unhealthy. Health predicts whether a node will still answer, so it is only
// meaningful for a node that has not answered yet; applying it to a completed node
// retroactively fails a round whose work is finished, returning ABORTED_BY_HEALTH
// despite every node reporting success. That makes the round unsatisfiable by
// construction whenever a node's fault outlasts its own work.
TEST(CompletionTrackerTest, UnhealthyNodeThatAlreadyReportedSuccessStillCompletes)
{
  CompletionTracker tracker([](const std::string &) -> std::optional<bool> {return false;});
  const auto id = make_uuid(7);
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  tracker.on_goal_accepted("em_a", id, round);
  tracker.on_result("em_a", id, true, 0, "", round);

  EXPECT_EQ(tracker.wait_for_all(1s), WaitResult::COMPLETE)
    << "a node that finished its work must not be failed retroactively for being unhealthy "
       "afterwards -- health only predicts whether an UNANSWERED node will answer";
}

// A node that reported FAILURE is still a failure, not a health abort: the more
// specific outcome must win, so the caller's log/alarm names what actually happened.
TEST(CompletionTrackerTest, UnhealthyNodeThatReportedFailureIsFailedNotHealthAborted)
{
  CompletionTracker tracker([](const std::string &) -> std::optional<bool> {return false;});
  const auto id = make_uuid(8);
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  tracker.on_goal_accepted("em_a", id, round);
  tracker.on_result("em_a", id, false, 42, "the real reason", round);

  EXPECT_EQ(tracker.wait_for_all(1s), WaitResult::FAILED)
    << "an explicit failure report must be reported as FAILED, not masked by the node's health";
}

// health_cross_check=false (what the manager passes for ABORTING) makes the wait ignore
// health entirely: an unhealthy node no longer aborts it, so only a real result or the
// timeout can end it. This is what stops ABORTING -- the terminal response to an unhealthy
// node -- from being failed by the very fault it exists to handle, with nowhere to escalate.
TEST(CompletionTrackerTest, HealthCrossCheckDisabledIgnoresUnhealthyNode)
{
  CompletionTracker tracker([](const std::string &) -> std::optional<bool> {return false;});
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});

  // With the cross-check ON this same setup returns ABORTED_BY_HEALTH immediately
  // (see HealthCrossCheckAbortsWaitFast); with it OFF the unhealthy node is ignored.
  EXPECT_EQ(tracker.wait_for_all(50ms, {}, /*health_cross_check=*/false), WaitResult::TIMEOUT)
    << "with the cross-check disabled an unhealthy node must not abort the wait";
}

// And with the cross-check off, a genuine completion still resolves the wait normally --
// disabling health must not disable completion.
TEST(CompletionTrackerTest, HealthCrossCheckDisabledStillCompletesOnRealReport)
{
  CompletionTracker tracker([](const std::string &) -> std::optional<bool> {return false;});
  const auto id = make_uuid(9);
  [[maybe_unused]] const auto round = tracker.begin_round({"em_a"});
  tracker.on_goal_accepted("em_a", id, round);
  tracker.on_result("em_a", id, true, 0, "", round);

  EXPECT_EQ(tracker.wait_for_all(1s, {}, /*health_cross_check=*/false), WaitResult::COMPLETE);
}

// ---------------------------------------------------------------------------
// Round identity. Rebuilding nodes_ cannot separate rounds by itself: every round rebuilds
// the same key set from the same immutable client map, so a late callback from a dead round
// still finds a live entry to write into.
// ---------------------------------------------------------------------------

// A ClientFanout deliberately outlives a round (its 5s deadline vs the round's much longer
// completion budget), so an OLD fan-out's expiry calls on_goal_rejected while a NEW round is
// live. Without round identity that spuriously fails the live round, aborting a transition
// whose Equipment Modules are answering correctly.
TEST(CompletionTrackerTest, StaleRoundRejectionDoesNotFailTheLiveRound)
{
  CompletionTracker tracker;
  const auto old_round = tracker.begin_round({"em_a"});

  // A new round supersedes it before the old fan-out's deadline expires.
  const auto live_round = tracker.begin_round({"em_a"});
  const auto id = make_uuid(1);
  tracker.on_goal_accepted("em_a", id, live_round);

  // The dead round's deadline now fires.
  tracker.on_goal_rejected("em_a", "action server not available", old_round);

  // The live round must be untouched -- still waiting, not failed.
  EXPECT_EQ(tracker.wait_for_all(50ms), WaitResult::TIMEOUT)
    << "a superseded round's rejection failed the live round";

  // And it still completes normally on its own node's real answer.
  tracker.on_result("em_a", id, true, 0, "", live_round);
  EXPECT_EQ(tracker.wait_for_all(1s), WaitResult::COMPLETE);
}

// A dead round's goal ACCEPTANCE must not register against the live round either: it would
// bind a stale goal id, after which the live round's real result is discarded as a mismatch
// and the transition rides out its full timeout for no reason.
TEST(CompletionTrackerTest, StaleRoundAcceptanceDoesNotBindAgainstLiveRound)
{
  CompletionTracker tracker;
  const auto old_round = tracker.begin_round({"em_a"});
  const auto stale_id = make_uuid(1);

  const auto live_round = tracker.begin_round({"em_a"});
  const auto live_id = make_uuid(2);

  // ORDER MATTERS: the live round binds its own goal id FIRST, and only then does the dead
  // round's acceptance arrive. That is the damaging order -- a stale acceptance arriving
  // before the live one is harmlessly overwritten, so testing it that way round proves
  // nothing (an earlier draft of this test did exactly that and passed without the guard).
  tracker.on_goal_accepted("em_a", live_id, live_round);
  tracker.on_goal_accepted("em_a", stale_id, old_round);
  tracker.on_result("em_a", live_id, true, 0, "", live_round);

  EXPECT_EQ(tracker.wait_for_all(1s), WaitResult::COMPLETE)
    << "the live round's real result was not accepted -- a stale acceptance had bound a "
       "different goal id over it";
}

// A dead round's RESULT must not satisfy the live round. The goal-id check already covered
// this, so it should hold both before and after the round id -- kept as a guard that adding
// round identity did not weaken the existing protection.
TEST(CompletionTrackerTest, StaleRoundResultDoesNotSatisfyLiveRound)
{
  CompletionTracker tracker;
  const auto old_round = tracker.begin_round({"em_a"});
  const auto stale_id = make_uuid(1);
  tracker.on_goal_accepted("em_a", stale_id, old_round);

  const auto live_round = tracker.begin_round({"em_a"});
  tracker.on_result("em_a", stale_id, true, 0, "", old_round);

  EXPECT_EQ(tracker.wait_for_all(50ms), WaitResult::TIMEOUT)
    << "a superseded round's result satisfied the live round";
  EXPECT_EQ(tracker.current_round(), live_round);
}

// begin_round() hands out strictly increasing ids, so "is this still the live round?" is a
// plain comparison for any holder of a longer-lived object.
TEST(CompletionTrackerTest, RoundIdsAreMonotonicAndReadableViaCurrentRound)
{
  CompletionTracker tracker;
  const auto r1 = tracker.begin_round({"em_a"});
  EXPECT_EQ(tracker.current_round(), r1);
  const auto r2 = tracker.begin_round({"em_a"});
  EXPECT_GT(r2, r1);
  EXPECT_EQ(tracker.current_round(), r2);
}

// A failure is found regardless of where the failing node sorts among the still-working ones.
//
// A predicate that returns false at the first node with no result yet never looks at a failure
// already recorded by a node LATER in the map. Because nodes_ is a std::map<std::string, Entry>,
// that makes node NAMES decide the outcome: a slow "arm" and a failed "gripper" produce TIMEOUT
// after the full state_complete_timeout_ms, while the same two roles with the names swapped
// produce FAILED immediately. Both orderings are asserted here, and the
// wait bound is deliberately long relative to the assertion so a regression shows up as a multi-
// second test rather than a subtly wrong enum.
TEST(CompletionTrackerTest, FailureIsFoundWhicheverNameTheFailingNodeHas)
{
  for (const auto & [failing, pending] : std::vector<std::pair<std::string, std::string>>{
      {"z_fails_sorts_last", "a_pending_sorts_first"},
      {"a_fails_sorts_first", "z_pending_sorts_last"}})
  {
    CompletionTracker tracker;
    [[maybe_unused]] const auto round = tracker.begin_round({failing, pending});
    tracker.on_goal_accepted(failing, make_uuid(1), round);
    tracker.on_goal_accepted(pending, make_uuid(2), round);
    tracker.on_result(failing, make_uuid(1), /*success=*/false, /*error_code=*/7,
      "gripper reported a fault", round);

    const auto start = std::chrono::steady_clock::now();
    const auto result = tracker.wait_for_all(3000ms);
    const auto elapsed = std::chrono::steady_clock::now() - start;

    EXPECT_EQ(result, WaitResult::FAILED)
      << "with '" << failing << "' failed and '" << pending << "' still working, the round did not "
      << "report FAILED -- the scan stopped before reaching the failure";
    EXPECT_LT(elapsed, 500ms)
      << "the failure was not seen promptly; the round rode out its timeout instead";
  }
}
