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

  CmdEvent(const TransitionCmd &cmd_value, std::shared_ptr<std::promise<bool>> p)
      : QEvent(QEvent::Type(PACKML_CMD_EVENT_TYPE)), cmd(cmd_value), prom(std::move(p)) {}

  TransitionCmd cmd;
  std::shared_ptr<std::promise<bool>> prom;
};
} // namespace packml_sm