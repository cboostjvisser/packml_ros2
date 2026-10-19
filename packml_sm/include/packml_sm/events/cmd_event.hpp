// Copyright (c) 2016 Shaun Edwards
// Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific (ROS 2
// compatibility)
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

#pragma once

#include <atomic>
#include <future>
#include <memory>

#include "QEvent"
#include "packml_sm/common.hpp"

namespace packml_sm {
static int PACKML_CMD_EVENT_TYPE = QEvent::User + 1;

struct CmdEvent : public QEvent {
  static CmdEvent *clear(std::shared_ptr<std::promise<bool>> p) { return new CmdEvent(TransitionCmd::CLEAR, std::move(p)); }
  static CmdEvent *start(std::shared_ptr<std::promise<bool>> p) { return new CmdEvent(TransitionCmd::START, std::move(p)); }
  static CmdEvent *stop(std::shared_ptr<std::promise<bool>> p) { return new CmdEvent(TransitionCmd::STOP, std::move(p)); }
  static CmdEvent *hold(std::shared_ptr<std::promise<bool>> p) { return new CmdEvent(TransitionCmd::HOLD, std::move(p)); }
  static CmdEvent *abort(std::shared_ptr<std::promise<bool>> p) { return new CmdEvent(TransitionCmd::ABORT, std::move(p)); }
  static CmdEvent *reset(std::shared_ptr<std::promise<bool>> p) { return new CmdEvent(TransitionCmd::RESET, std::move(p)); }
  static CmdEvent *suspend(std::shared_ptr<std::promise<bool>> p) { return new CmdEvent(TransitionCmd::SUSPEND, std::move(p)); }
  static CmdEvent *unsuspend(std::shared_ptr<std::promise<bool>> p) { return new CmdEvent(TransitionCmd::UNSUSPEND, std::move(p)); }
  static CmdEvent *unhold(std::shared_ptr<std::promise<bool>> p) { return new CmdEvent(TransitionCmd::UNHOLD, std::move(p)); }

  // Born REFUSED. A QEvent is constructed accepted, Qt never reads or clears that flag itself,
  // and the only thing that lowers it is a transition's eventTest() calling ignore(). So the flag
  // means "no transition objected", not "a transition claimed this" -- and in a state where no
  // command transition is consulted at all, every command reports success. ABORTING is such a
  // state: it hangs off the machine rather than off `abortable`, so the only edges consulted are
  // its own completion and error edges, and both return on the event-type check before reaching
  // an ignore(). Left at Qt's default, start(), clear() and stop() issued in ABORTING each return
  // true with the machine unmoved, and the ROS service reports success.
  CmdEvent(const TransitionCmd &cmd_value, std::shared_ptr<std::promise<bool>> p)
      : QEvent(QEvent::Type(PACKML_CMD_EVENT_TYPE)), cmd(cmd_value), prom(std::move(p))
  {
    setAccepted(false);
  }

  // A command that is discarded is a command that was refused. Qt deletes every event it owns --
  // those it processes, and those still queued when the machine is restarted or destroyed -- so
  // this destructor is the one point every never-to-be-evaluated command passes through. Without
  // an answer here the caller's promise dies unfulfilled and its f.get() throws std::future_error
  // on whichever thread issued the command; for the manager node that is a ROS service callback
  // with no handler above it.
  ~CmdEvent() override { answer(false); }

  // At most one answer ever reaches the caller. Transition selection and this event's own
  // destruction both offer one, and set_value() on an already-satisfied promise throws -- from
  // inside Qt's event loop, where an escaping exception terminates the process.
  void answer(bool accepted)
  {
    if (prom && !answered.exchange(true, std::memory_order_acq_rel)) {
      prom->set_value(accepted);
    }
  }

  TransitionCmd cmd;
  std::shared_ptr<std::promise<bool>> prom;

private:
  std::atomic<bool> answered{false};
};
} // namespace packml_sm