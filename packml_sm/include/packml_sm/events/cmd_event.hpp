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

#include "QEvent"
#include "packml_sm/common.hpp"

namespace packml_sm {
static int PACKML_CMD_EVENT_TYPE = QEvent::User + 1;

struct CmdEvent : public QEvent {
  static CmdEvent *clear() { return new CmdEvent(TransitionCmd::CLEAR); }
  static CmdEvent *start() { return new CmdEvent(TransitionCmd::START); }
  static CmdEvent *stop() { return new CmdEvent(TransitionCmd::STOP); }
  static CmdEvent *hold() { return new CmdEvent(TransitionCmd::HOLD); }
  static CmdEvent *abort() { return new CmdEvent(TransitionCmd::ABORT); }
  static CmdEvent *reset() { return new CmdEvent(TransitionCmd::RESET); }
  // static CmdEvent *estop() { return new CmdEvent(Cmd::ESTOP); }
  static CmdEvent *suspend() { return new CmdEvent(TransitionCmd::SUSPEND); }
  static CmdEvent *unsuspend() { return new CmdEvent(TransitionCmd::UNSUSPEND); }
  static CmdEvent *unhold() { return new CmdEvent(TransitionCmd::UNHOLD); }

  explicit CmdEvent(const TransitionCmd &cmd_value)
      : QEvent(QEvent::Type(PACKML_CMD_EVENT_TYPE)), cmd(cmd_value) {}

  TransitionCmd cmd;
};
} // namespace packml_sm