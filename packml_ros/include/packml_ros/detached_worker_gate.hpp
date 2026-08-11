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

#pragma once

#include <condition_variable>
#include <mutex>

namespace packml_ros {

/// Counts the detached threads that are currently inside an object, so that object can refuse to
/// finish being destroyed while one of them is still using it.
///
/// Both places this package detaches a thread hand it `this` and then keep using the object for a
/// long time: the manager's mode-change fan-out (its own wait alone is up to 200 ms) and an
/// equipment module's deferred-completion wait (as long as its configured timeout, realistically
/// seconds). Detaching means nothing joins them, so destroying the owner inside either window --
/// a launch shutdown, a Ctrl-C, a supervisor restart -- left a live thread working through freed
/// memory. Two tests carry deliberate multi-second settle sleeps that exist only to usually miss
/// that.
///
/// Deliberately a counter rather than a list of thread handles: the threads are detached precisely
/// because nothing wants to own their handles, and the only question an owner needs answered on
/// teardown is "is anybody still inside me".
///
/// Pairs with a shutting-down flag on the owner. This gate makes teardown WAIT for a worker; it is
/// the owner's job to also make that worker give up promptly, or the wait is as long as whatever
/// the worker was waiting for.
class DetachedWorkerGate
{
public:
  /// Call on the owning thread BEFORE spawning, never inside the new thread -- otherwise teardown
  /// can slip between the spawn and the count going up, which is the whole window being closed.
  void enter()
  {
    std::lock_guard<std::mutex> lk(count_mutex_);
    ++count_;
  }

  /// The notify happens WHILE the lock is held, which is the opposite of the usual advice and is
  /// required here. The usual advice optimises for a woken waiter not immediately blocking on the
  /// mutex; it assumes the condition_variable outlives the notify. This one does not: the waiter is
  /// a destructor. Release the lock first and await_idle() can observe count_ == 0, return, and let
  /// the owner finish destroying itself -- taking count_mutex_ and idle_ with it -- while this
  /// thread is still inside notify_all() touching both. That window is wide and needs no unlucky
  /// interleaving at all -- an awaiter arriving after the decrement and before the notify finds
  /// the predicate already true and never blocks.
  ///
  /// Holding the lock across the notify closes it: a waiter cannot return from wait() until it
  /// re-acquires count_mutex_, which cannot happen until this scope ends, by which point
  /// notify_all() has returned. What remains is the ordinary and unavoidable one -- the unlock is
  /// the last touch of a mutex the waiter may destroy immediately after acquiring and releasing
  /// it -- and no arrangement of a mutex-based gate removes that.
  void leave()
  {
    std::lock_guard<std::mutex> lk(count_mutex_);
    --count_;
    idle_.notify_all();
  }

  /// Block until every worker that entered has left.
  void await_idle()
  {
    std::unique_lock<std::mutex> lk(count_mutex_);
    idle_.wait(lk, [this] {return 0 == count_;});
  }

  /// Leaves the gate however the worker returns. The worker bodies have several exit paths each,
  /// so this is not a convenience.
  class Scope
  {
public:
    explicit Scope(DetachedWorkerGate & gate)
    : gate_(gate) {}
    ~Scope() {gate_.leave();}
    Scope(const Scope &) = delete;
    Scope & operator=(const Scope &) = delete;

private:
    DetachedWorkerGate & gate_;
  };

private:
  std::mutex count_mutex_;
  std::condition_variable idle_;
  int count_{0};
};

}  // namespace packml_ros
