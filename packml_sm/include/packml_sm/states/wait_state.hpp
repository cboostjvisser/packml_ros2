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
#include "packml_sm/common.hpp"
#include "packml_sm/states/state.hpp"
#include <qchar.h>

namespace packml_sm
{

struct WaitState : public PackmlState
{
  static WaitState * Idle(QState * stoppable)
  {
    return new WaitState(State::IDLE, TransitionCmd::START, QString(to_string(State::IDLE).c_str()), stoppable);
  }
  static WaitState * Held(QState * stoppable)
  {
    return new WaitState(State::HELD, TransitionCmd::UNHOLD, QString(to_string(State::HELD).c_str()), stoppable);
  }
  static WaitState * Complete(QState * stoppable)
  {
    return new WaitState(State::COMPLETE, TransitionCmd::RESET, QString(to_string(State::COMPLETE).c_str()), stoppable);
  }
  static WaitState * Suspended(QState * stoppable)
  {
    return new WaitState(State::SUSPENDED, TransitionCmd::UNSUSPEND, QString(to_string(State::SUSPENDED).c_str()), stoppable);
  }
  static WaitState * Stopped(QState * abortable)
  {
    return new WaitState(State::STOPPED, TransitionCmd::RESET, QString(to_string(State::STOPPED).c_str()), abortable);
  }
  static WaitState * Aborted()
  {
    return new WaitState(State::ABORTED, TransitionCmd::CLEAR, QString(to_string(State::ABORTED).c_str()));
  }

  WaitState(State state_value, TransitionCmd exit_cmd_value, QString name_value)
    : PackmlState(state_value, name_value), exit_cmd(exit_cmd_value) {}

  WaitState(State state_value, TransitionCmd exit_cmd_value, QString name_value, QState * super_state)
    : PackmlState(state_value, name_value, super_state), exit_cmd(exit_cmd_value) {}

  virtual ~WaitState() {}

private:
  TransitionCmd exit_cmd;
};
}  // namespace packml_sm