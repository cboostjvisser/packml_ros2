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

#include "QEvent"


#include "packml_sm/events/cmd_event.hpp"
#include "packml_sm/transitions/cmd_transition.hpp"

namespace packml_sm {
CmdTransition::CmdTransition(const TransitionCmd &cmd_value,
                             const QString &name_value, PackmlState &from,
                             PackmlState &to)
    : cmd(cmd_value), name(name_value) {
  // this->setTargetState(&to);
  // from.addTransition(this);
  std::cout << "Creating " << this->name.toStdString() << " transition from "
            << from.name() << " to " << to.name()
            << std::endl;
}

bool CmdTransition::eventTest(QEvent *e) {
  //    ROS_INFO_STREAM("Testing event type: " << e->type());
  if (e->type() != QEvent::Type(PACKML_CMD_EVENT_TYPE)) {
    return false;
  }
  CmdEvent *se = static_cast<CmdEvent *>(e);
  //    ROS_INFO_STREAM("Type cmd: " << cmd << ", event cmd: " << se->cmd);
  if (cmd == se->cmd) {
    e->accept();
    return true;
  }

  std::cout << "Received transition command: " << cmd
            << " on transition: " << this->name.toStdString() << std::endl;

  std::cout << "Event is not for this transition" << std::endl;

  e->ignore();

  return false;
}

} // namespace packml_sm
