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

#pragma once

#include "packml_sm/common.hpp"
#include "packml_sm/states/state.hpp"

namespace packml_sm
{

struct PackmlSuperState : public PackmlState
{
public:
  static PackmlSuperState * Abortable()
  {
    return new PackmlSuperState(SuperState::ABORTABLE, TransitionCmd::ABORT, "Abortable");
  }
  static PackmlSuperState * Stoppable(QState * superstate)
  {
    return new PackmlSuperState(SuperState::STOPPABLE, TransitionCmd::STOP, "Stoppable", superstate);
  }

  PackmlSuperState(SuperState state_value, TransitionCmd transition_command, QString name_value, QState * super_state = nullptr)
    : PackmlState(State::UNDEFINED, name_value, super_state), transition_command(transition_command) {}

private:
  TransitionCmd transition_command;
};
}  // namespace packml_sm