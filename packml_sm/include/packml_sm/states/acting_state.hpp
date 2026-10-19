// Copyright (c) 2016 Shaun Edwards
// Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific (ROS 2 compatibility)
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

#pragma  once

#include "QState"
#include "QFuture"
#include "packml_sm/common.hpp"
#include "packml_sm/states/state.hpp"
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <qchar.h>
#include <stop_token>

class QThreadPool;

namespace packml_sm
{

struct ActingState : public PackmlState
{
public:
  static ActingState * Resetting(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::RESETTING, stoppable, delay_ms_value);
  }

  static ActingState * Starting(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::STARTING, stoppable, delay_ms_value);
  }

  static ActingState * Unholding(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::UNHOLDING, stoppable, delay_ms_value);
  }

  static ActingState * Unsuspending(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::UNSUSPENDING, stoppable, delay_ms_value);
  }

  static ActingState * Holding(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::HOLDING, stoppable, delay_ms_value);
  }

  static ActingState * Suspending(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::SUSPENDING, stoppable, delay_ms_value);
  }

  static ActingState * Execute(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::EXECUTE, stoppable, delay_ms_value);
  }

  static ActingState * Execute(QState * stoppable, std::function<int()> function_value)
  {
    return new ActingState(State::EXECUTE, stoppable, function_value);
  }

  static ActingState * Completing(QState * stoppable, int delay_ms_value = 200)
  {
    return new ActingState(State::COMPLETING, stoppable, delay_ms_value);
  }

  static ActingState * Aborting(int delay_ms_value = 200)
  {
    return new ActingState(State::ABORTING, delay_ms_value);
  }

  static ActingState * Clearing(QState * abortable, int delay_ms_value = 200)
  {
    return new ActingState(State::CLEARING, abortable, delay_ms_value);
  }

  static ActingState * Stopping(QState * abortable, int delay_ms_value = 200)
  {
    return new ActingState(State::STOPPING, abortable, delay_ms_value);
  }

  ActingState(State state_value, const char * name_value, int delay_ms_value = 200)
    : PackmlState(state_value, QString(name_value)), delay_ms(delay_ms_value) {}

  ActingState(State state_value, const QString & name_value, QState * super_state, int delay_ms_value = 200)
    : PackmlState(state_value, name_value, super_state), delay_ms(delay_ms_value) {}

  ActingState(State state_value, const char * name_value, QState * super_state, std::function<int()> function_value)
    : PackmlState(state_value, QString(name_value), super_state), function_(function_value) {}


  ActingState(State state_value, int delay_ms_value = 200)
    : PackmlState(state_value, to_string(state_value).c_str()), delay_ms(delay_ms_value) {}

  ActingState(State state_value, QState * super_state, int delay_ms_value = 200)
    : PackmlState(state_value, to_string(state_value).c_str(), super_state), delay_ms(delay_ms_value) {}

  ActingState(State state_value, QState * super_state, std::function<int()> function_value)
    : PackmlState(state_value, to_string(state_value).c_str(), super_state), function_(function_value) {}

  // Where a failure of THIS state's own bound operation must land. Every acting state has one,
  // declared once when the graph is wired (StatesGenerator::generate_all_packml_states) and
  // paired there with the ERROR transition that reaches it. An acting state whose bound
  // operation can fail but whose failure has no declared destination wedges the machine in
  // silence, because Qt discards an event that matches no transition.
  void declareFailureTarget(PackmlState * target)
  {
    failure_target_ = target;
  }

  PackmlState * failureTarget() const
  {
    return failure_target_;
  }

  bool setOperationMethod(std::function<int()> function_value)
  {
    function_ = function_value;
    interruptible_function_ = nullptr;
    return true;
  }

  /// Has anyone bound real work to this state?
  bool hasBoundOperation() const
  {
    return nullptr != function_ || nullptr != interruptible_function_;
  }

  /// With nothing bound, hold in this state until it is commanded out, instead of falling back
  /// to the delay-then-complete default.
  ///
  /// Set for EXECUTE in a continuous-cycle machine, and the reason is the difference between a
  /// tick and a batch boundary. For every other acting state, "no bound operation" sensibly means
  /// "this step takes delay_ms and then it is done". For EXECUTE it does not: leaving EXECUTE is
  /// the end of production, which the standard reaches either when the product counter hits its
  /// limit or on an explicit Complete command -- never on a timer. A machine with nothing bound
  /// has produced nothing, so completing on a timer fabricates a batch boundary that nothing
  /// asked for, and every one of them costs a full fan-out round to every equipment module.
  void holdWhenUnbound()
  {
    hold_when_unbound_ = true;
  }

  bool holdsWhenUnbound() const
  {
    return hold_when_unbound_;
  }

  /// Run this state's bound operations in `pool` instead of the global QtConcurrent pool.
  ///
  /// Set by the owning StateMachine so that machine can wait for exactly its own operations on
  /// teardown, and so their threads come out of its own budget. Left null, operations fall back to
  /// the global pool, which is what a bare ActingState outside a StateMachine gets.
  void setOperationThreadPool(QThreadPool * pool)
  {
    operation_pool_ = pool;
  }
  // Like setOperationMethod(), but the bound function receives a std::stop_token that
  // becomes stop_requested() the moment this state is asked to exit EARLY -- i.e. a
  // transition out of this state (e.g. an operator's HOLD/SUSPEND/ABORT/STOP) was
  // accepted while the function was still running. A function that observes the token
  // can return promptly instead of riding out its own internal timeout while the state
  // machine sits blocked waiting for it (see onExit()). Purely additive: existing
  // callers of setOperationMethod()/setExecute()/setResetting() are unaffected.
  bool setInterruptibleOperationMethod(std::function<int(std::stop_token)> function_value)
  {
    interruptible_function_ = function_value;
    function_ = nullptr;
    return true;
  }
  // Is the calling thread inside some acting state's bound operation?
  //
  // Neither the not-running guard nor the event-loop-thread guard in
  // StateMachine::postCommand() covers this caller: the machine IS running, and a thread-pool
  // worker is not the loop thread.
  //
  // Needed because a bound operation that blocks on the machine holds run_mutex_ while it
  // waits, starving every later entry to the same state behind it, and because there is already a
  // correct way to do what such a caller wants: return a non-zero code and the resulting ErrorEvent
  // escalates to this state's declared failure target without blocking anyone.
  static bool callerIsBoundOperation();

  /// Ask this state's in-flight bound operation to stop, without waiting for it. Safe from any
  /// thread.
  ///
  /// Exists because QStateMachine::stop() does NOT call onExit() on active states, so the
  /// deactivate() path had no way to request the stop that onExit() requests on every other
  /// path. Without it, a bound operation running when the machine stops finished normally and
  /// posted its StateCompleteEvent into a stopped machine, which Qt warns about, drops, and does
  /// not delete -- and the drain waited out the full operation instead of the shortened one an
  /// interruptible function would have given it.
  void requestOperationStop();

  virtual void operation();

  // Block until every worker this state has started has returned.
  //
  // Required at deactivate/destroy time because QStateMachine::stop() does NOT invoke onExit() on
  // currently active states, so without this a worker can outlive the state machine and post an
  // event to a destroyed machine().
  //
  // Waits on a counter rather than on a QFuture. A QFuture here is ASSIGNED by onEntry() on the Qt
  // thread while this call reads it from another, and QFuture is reentrant, not thread-safe --
  // re-entering the state rewrites the interface pointer under the waiter and segfaults in
  // QMutex::unlock. A counter guarded by the same mutex the waiter holds has no such window.
  void waitForOperationFinished()
  {
    std::unique_lock<std::mutex> lk(workers_mutex_);
    workers_idle_.wait(lk, [this] { return 0 == outstanding_workers_; });
  }
  virtual ~ActingState() {}

protected:
  virtual void onEntry(QEvent * e);
  virtual void onExit(QEvent * e);

private:
  // The body of operation(), taking the activation to stamp on whatever event it posts and the
  // stop token belonging to the entry that started it. Both are captured INTO the worker task by
  // onEntry() rather than read back off this object once the bound function has returned -- by
  // which time a subsequent entry could have moved the activation and, worse, REPLACED
  // stop_source_ underneath the read. The token is a value tied to its own entry's source, so a
  // worker outliving its visit still asks the right question.
  void run_operation(uint64_t entry_activation, std::stop_token token);

  int delay_ms;
  PackmlState * failure_target_{nullptr};
  std::function<int()> function_;
  std::function<int(std::stop_token)> interruptible_function_;

  // How many workers this state has started and not yet seen return, and the gate
  // waitForOperationFinished() waits on. Replaces a QFuture that was assigned on the Qt thread
  // and waited on from another; see waitForOperationFinished().
  std::mutex workers_mutex_;
  std::condition_variable workers_idle_;
  int outstanding_workers_{0};

  // Held for the duration of the bound function, so at most one runs per state at a time even
  // though onExit() does not join. Taken on the WORKER thread, never on the Qt thread: that is
  // the whole point -- serialising the operations must not serialise the event loop.
  std::mutex run_mutex_;
  // Fresh per-entry (see onEntry()): lets onExit() tell an in-flight bound function
  // "you're being exited early" without needing the caller to have wired anything up
  // itself. Requesting stop on a function that never observes the token (the plain
  // setOperationMethod() case) is harmless -- nothing polls it -- but operation()
  // still uses it to decide whether to suppress a now-stale completion/error event
  // either way; see operation()'s own comment.
  //
  // Guarded, because it has three writers on two threads: onEntry() replaces it and onExit()
  // requests stop on the Qt thread, and requestOperationStop() requests stop from whichever
  // thread is deactivating. Replacing a std::stop_source while another thread calls request_stop()
  // on it is the same shape of race that the QFuture this class used to hold died of. Workers do
  // not take this lock at all -- they carry a std::stop_token copied at entry, which is
  // thread-safe against request_stop() by design.
  std::mutex stop_mutex_;
  std::stop_source stop_source_;

  // See holdWhenUnbound().
  bool hold_when_unbound_{false};

  // See setOperationThreadPool(). Written once, before the machine is activated, and read on the
  // Qt thread at every entry -- so no synchronisation, for the same reason delay_ms needs none.
  QThreadPool * operation_pool_{nullptr};
};

// TODO: needed?
typedef ActingState DualState;


}  // namespace packml_sm