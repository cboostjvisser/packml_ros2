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
// Anonymous completion and error events -- matched on QEvent type alone -- let any
// StateCompleteEvent satisfy any armed StateCompleteTransition.  A bound operation runs on a
// worker thread and the event it posts is queued, so the machine can leave the state -- or leave
// and re-enter it -- before Qt processes what that state said.  Whatever transition happened to be
// armed by then consumed it.
//
// These tests drive that directly rather than trying to lose a race: they hand-post events with
// chosen tokens against a purpose-built three-state graph, so each hazard is one deterministic
// step.  In the real machine the same hazards need a specific interleaving to appear, which is
// exactly why they went unnoticed for so long and why pinning them here rather than through the
// full PackML graph is the point.
// ---

#include <chrono>
#include <string>
#include <thread>

#include <QCoreApplication>
#include <gtest/gtest.h>

#include "packml_sm/common.hpp"
#include "packml_sm/events/activation_token.hpp"
#include "packml_sm/events/sc_event.hpp"
#include "packml_sm/state_machine.hpp"
#include "packml_sm/states/state.hpp"
#include "packml_sm/states/wait_state.hpp"
#include "packml_sm/transitions/sc_transition.hpp"

namespace {

using packml_sm::ActivationToken;
using packml_sm::PackmlState;
using packml_sm::PackmlStateMachine;
using packml_sm::StateCompleteEvent;
using packml_sm::StateCompleteTransition;
using packml_sm::TransitionCmd;
using packml_sm::WaitState;

// A graph of plain wait states wired only with completion transitions, so nothing posts events of
// its own and every event under test is one the test put there.  Alpha -> Beta -> Gamma, plus a
// Beta self-loop, which is the shape that makes a state re-enterable while its own earlier
// completion is still in flight (EXECUTE under ContinuousCycle, and RESETTING from either STOPPED
// or COMPLETE, are the real instances).
struct Rig
{
  PackmlStateMachine machine;
  WaitState * alpha;
  WaitState * beta;
  WaitState * gamma;

  Rig()
  {
    // States reuse PackML enum values only so they print recognisably; the graph is not PackML.
    alpha = new WaitState(packml_sm::State::IDLE, TransitionCmd::START, "alpha");
    beta = new WaitState(packml_sm::State::EXECUTE, TransitionCmd::HOLD, "beta");
    gamma = new WaitState(packml_sm::State::COMPLETE, TransitionCmd::RESET, "gamma");

    auto * alpha_beta = new StateCompleteTransition();
    alpha_beta->setTargetState(beta);
    alpha->addTransition(alpha_beta);

    // Beta's own completion loops back to Beta, and only a COMMAND leaves for Gamma.  So a
    // completion consumed at Beta is always visible as a re-entry, never as forward progress.
    auto * beta_beta = new StateCompleteTransition();
    beta_beta->setTargetState(beta);
    beta->addTransition(beta_beta);

    machine.addState(alpha);
    machine.addState(beta);
    machine.addState(gamma);
    machine.setInitialState(alpha);

    machine.moveToThread(QCoreApplication::instance()->thread());
    machine.start();
  }

  ~Rig()
  {
    machine.stop();
    // Let the stop actually land before the states are destroyed with the machine.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  void post(const ActivationToken & token)
  {
    machine.postEvent(new StateCompleteEvent(token));
  }

  // Wait until the machine settles: both that it reached `expected` and that it stayed there.
  bool settled_in(const PackmlState * expected)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      if (machine.configuration().contains(const_cast<PackmlState *>(expected))) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return machine.configuration().contains(const_cast<PackmlState *>(expected));
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }
};

}  // namespace

// A completion posted by one state must not be consumed as another state's completion.
//
// This is the hazard that mattered most in the real graph: ABORTING is where every fault is
// funnelled, and a stale completion from whichever state the abort interrupted satisfied ABORTING's
// own completion edge, carrying the machine straight to ABORTED without the abort operation ever
// having run.
TEST(EventIdentity, CompletionFromAnotherStateIsNotConsumedHere)
{
  Rig rig;
  ASSERT_TRUE(rig.settled_in(rig.alpha));

  // Alpha's genuine completion: accepted, so the machine advances and Alpha's visit ends.
  rig.post(ActivationToken{rig.alpha, rig.alpha->activation()});
  ASSERT_TRUE(rig.settled_in(rig.beta));
  const auto beta_visit = rig.beta->activation();

  // Now a completion from Alpha arrives while Beta is the one running.  Beta's completion edge is
  // armed and, on a type-only test, fires -- crediting Beta with work Alpha reported.
  rig.post(ActivationToken{rig.alpha, 1});
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(rig.beta->activation(), beta_visit)
    << "Beta was re-entered, so it consumed a completion that Alpha posted";
}

// A completion posted on one visit to a state must not be consumed on a later visit to it.
//
// Attribution alone does not catch this one: the origin IS the source, so only the visit number
// tells the ghost apart.  Reachable wherever a state can be left and re-entered while its own
// earlier completion is still queued.
TEST(EventIdentity, CompletionFromAnEarlierVisitIsNotConsumedOnALaterOne)
{
  Rig rig;
  ASSERT_TRUE(rig.settled_in(rig.alpha));
  rig.post(ActivationToken{rig.alpha, rig.alpha->activation()});
  ASSERT_TRUE(rig.settled_in(rig.beta));

  const auto first_visit = rig.beta->activation();

  // Beta's own completion for the visit it is on: accepted, so Beta exits and re-enters itself.
  rig.post(ActivationToken{rig.beta, first_visit});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (rig.beta->activation() == first_visit &&
         std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const auto second_visit = rig.beta->activation();
  ASSERT_NE(second_visit, first_visit) << "the genuine completion was not accepted";

  // The same token again -- a duplicate of a completion whose visit is over.  On a type-only test
  // it loops Beta a second time on work that was already accounted for.
  rig.post(ActivationToken{rig.beta, first_visit});
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(rig.beta->activation(), second_visit)
    << "Beta was re-entered on a completion stamped with a visit that had already ended";
}

// A completion that cannot say which state finished must not be credited to whichever state is
// active.  Nothing in the tree mints one -- an acting state's bound operation is the only producer
// of completion events, and it always stamps -- so this pins the requirement rather than a bug:
// a future producer that forgets to stamp fails loudly here instead of quietly aliasing.
TEST(EventIdentity, UnattributedCompletionIsRefused)
{
  Rig rig;
  ASSERT_TRUE(rig.settled_in(rig.alpha));
  const auto alpha_visit = rig.alpha->activation();

  rig.machine.postEvent(new StateCompleteEvent());
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  EXPECT_EQ(rig.alpha->activation(), alpha_visit)
    << "an unattributed completion moved the machine out of Alpha";
}
