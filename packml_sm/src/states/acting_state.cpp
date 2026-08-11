// Copyright (c) 2016 Shaun Edwards
// Copyright (c) 2019 Dejanira Araiza Illan, ROS-Industrial Asia Pacific
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include <thread>

#include <QEvent>
#include <QThreadPool>
#include <QtConcurrent/QtConcurrent>

#include "packml_sm/logging.hpp"
#include "packml_sm/states/acting_state.hpp"

#include "packml_sm/events/sc_event.hpp"
#include "packml_sm/events/error_event.hpp"

namespace packml_sm {

namespace {
// Set only for the span of the bound function itself, not the whole worker task: the event this
// state posts afterwards is a post, not a wait, and nothing joins the worker once it has been
// posted. See ActingState::callerIsBoundOperation().
thread_local bool t_running_bound_operation = false;

struct BoundOperationScope
{
  BoundOperationScope() { t_running_bound_operation = true; }
  ~BoundOperationScope() { t_running_bound_operation = false; }
};
}  // namespace

bool ActingState::callerIsBoundOperation()
{
  return t_running_bound_operation;
}

void ActingState::onEntry(QEvent * e)
{
  PackmlState::onEntry(e);
  printf("Starting thread for state operation\n");
  std::stop_token entry_token;
  {
    std::lock_guard<std::mutex> lk(stop_mutex_);
    stop_source_ = std::stop_source{};  // fresh token for this entry
    entry_token = stop_source_.get_token();
  }
  {
    std::lock_guard<std::mutex> lk(workers_mutex_);
    ++outstanding_workers_;
  }
  // The activation is captured into the task rather than read from the object when the operation
  // finishes. Reading it later is only correct if something joins this worker before
  // PackmlState::onExit() can bump the count; nothing does, so a worker from the previous visit
  // would read the NEW count and stamp a stale event as current. The capture is what makes
  // running without that join safe.
  QtConcurrent::run(
    operation_pool_ ? operation_pool_ : QThreadPool::globalInstance(),
    [this, entry_activation = activation(), entry_token]() {
      run_operation(entry_activation, entry_token);
      {
        std::lock_guard<std::mutex> lk(workers_mutex_);
        --outstanding_workers_;
      }
      workers_idle_.notify_all();
    });
}

void ActingState::onExit(QEvent * e)
{
  // Ask the bound operation to stop, and DO NOT wait for it.
  //
  // Waiting here runs on the Qt thread, so the event loop would stop for as long as the operation
  // does: the command that triggers the transition is answered immediately and the next command
  // queued behind it waits out the whole operation -- the loop, not the transport, is what a
  // queued command waits for. Nothing here needs the operation's result: the interrupting
  // transition has already been selected, and run_operation() suppresses the event a stopped
  // operation would have posted.
  //
  // Worker lifetime is NOT this function's job: it belongs to waitForOperationFinished(), which
  // deactivate() and ~StateMachine() call before the machine can be destroyed. Stale events are
  // step 8's activation token.
  printf("State exit requested, asking the bound operation to stop\n");
  requestOperationStop();
  PackmlState::onExit(e);
}

void ActingState::requestOperationStop()
{
  std::lock_guard<std::mutex> lk(stop_mutex_);
  stop_source_.request_stop();
}

void ActingState::operation()
{
  std::stop_token token;
  {
    std::lock_guard<std::mutex> lk(stop_mutex_);
    token = stop_source_.get_token();
  }
  run_operation(activation(), token);
}

void ActingState::run_operation(uint64_t entry_activation, std::stop_token token)
{
  // The three things a run of the bound operation can mean to the state machine. Naming them is
  // the point: spread across early returns and a pair of `if`s, the routing hides cases -- a
  // failure with nowhere to go from ABORTING reads as ordinary control flow.
  //
  // SUPPRESSED is deliberately NOT something a bound function can ask for. It is reachable only
  // when the machine has ALREADY decided to leave this state, which is what makes it safe: a
  // suppressed run posts nothing, and nothing else will move the machine out of an acting state.
  // Hand that choice to a caller and a bound function that returns "no event" while nobody is
  // exiting wedges its state permanently -- the very failure class the rest of this work removes,
  // coming back through a new door.
  enum class Outcome { COMPLETED, FAILED, SUPPRESSED };

  int error_code = 0;

  // At most one bound function per state runs at a time. onExit() does not join, so a re-entry
  // can start a second worker while the first is still inside the user's function; serialising
  // here keeps that invisible to the bound function, which was never written to be reentrant.
  // Taken on this worker thread and never on the Qt thread -- serialising the operations must not
  // serialise the event loop, which is the whole point of not joining.
  std::lock_guard<std::mutex> run_lk(run_mutex_);

  if (interruptible_function_) {
    printf("Executing interruptible operational function in acting state\n");
    BoundOperationScope scope;
    error_code = interruptible_function_(token);
  } else if (function_) {
    printf("Executing operational function in acting state\n");
    BoundOperationScope scope;
    error_code = function_();
  } else if (hold_when_unbound_) {
    // No bound operation, and this state must not invent a completion for one (see
    // holdWhenUnbound()). Hold until the state is commanded out; the stop request that ends this
    // wait is also what makes the outcome below SUPPRESSED, so nothing is posted.
    PACKML_INFO_STREAM("packml_sm", "No operation bound to " << name() <<
      "; holding until commanded out");
    while (!token.stop_requested()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  } else {
    PACKML_INFO_STREAM("packml_sm", "Default operation, delaying " << delay_ms << " ms");
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    printf("Operation delay complete\n");
  }

  // This state was asked to exit EARLY (onExit() requested it) while the bound function was still
  // running, so whatever it just returned is racing an already-decided transition. Say nothing:
  // the interrupting transition needs no help from this one. The default timer is no exception.
  //
  // The activation stamped on the events below already makes a stale event harmless on its own,
  // so this is not the only thing standing between a late completion and the wrong state's
  // successor -- but it still saves the round trip, and it is the one case where posting nothing
  // is unambiguously right rather than merely safe.
  const Outcome outcome = token.stop_requested()
    ? Outcome::SUPPRESSED
    : (0 == error_code ? Outcome::COMPLETED : Outcome::FAILED);

  if (Outcome::SUPPRESSED == outcome) {
    return;
  }

  // A stopped machine consumes nothing and deletes nothing: QStateMachine::postEvent() on it warns,
  // returns, and leaves the event allocated. deactivate() asks every acting state to stop before
  // and after stopping the machine, so a worker that gets here with a stop request already pending
  // took the branch above; this covers the remainder of that window -- the machine stopping between
  // that check and this line. Dropping the event is what a stopped machine would have done with it
  // anyway, minus the warning and the leak.
  if (!machine()->isRunning()) {
    PACKML_INFO_STREAM("packml_sm", "Machine stopped while " << name() <<
      "'s operation was in flight; dropping the event it would have posted");
    return;
  }

  const ActivationToken activation_token{this, entry_activation};

  switch (outcome) {
    case Outcome::SUPPRESSED:
      return;
    case Outcome::COMPLETED:
      machine()->postEvent(new StateCompleteEvent(activation_token));
      return;
    case Outcome::FAILED:
      PACKML_WARN_STREAM("packml_sm", "Operational function returned error code: " << error_code <<
        "; escalating to " <<
        (nullptr != failure_target_ ? failure_target_->name() : std::string("<no declared target>")));
      machine()->postEvent(new ErrorEvent(error_code, activation_token));
      return;
  }
}

} // namespace packml_sm
