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

#include "packml_sm/logging.hpp"
#include "packml_sm/events/cmd_event.hpp"
#include "packml_sm/transitions/cmd_transition.hpp"

namespace packml_sm {
CmdTransition::CmdTransition(const TransitionCmd &cmd_value,
                             const QString &name_value, PackmlState &from,
                             PackmlState &to)
    : cmd(cmd_value), name(name_value) {
  this->setTargetState(&to);
  from.addTransition(this);
  PACKML_INFO_STREAM("packml_sm", "Creating " << this->name.toStdString() << " transition from "
            << from.name() << " to " << to.name());
}

bool CmdTransition::eventTest(QEvent *e) {
  //    ROS_INFO_STREAM("Testing event type: " << e->type());
  // std::cout << "Event type: " << e->type() << std::endl;
  if (e->type() != QEvent::Type(PACKML_CMD_EVENT_TYPE)) {
    return false;
  }
  CmdEvent *se = static_cast<CmdEvent *>(e);

  PACKML_DEBUG_STREAM("packml_sm", "Received transition command: " << se->cmd
            << " on transition: " << this->name.toStdString());

  // Whose command this is comes first. Asking the mode mask about a command this transition was
  // never for recorded a mask refusal against it -- so every unrelated command in flight looked,
  // in the log, like one the mode had blocked.
  //
  // Returning without ignore() is only safe because CmdEvent is constructed already refused, so
  // accept() below is the single thing that can mark a command claimed. Restore the event's
  // born-accepted default and this early return silently reports every command as successful.
  if (cmd != se->cmd) {
    PACKML_DEBUG_STREAM("packml_sm", "Event is not for this transition");
    return false;
  }

  // A mode declares which states an operator may command the machine INTO, so this is the
  // one transition kind that consults the mask. See target_available_in_current_mode().
  if (!target_available_in_current_mode(e)) {
    return false;
  }

  e->accept();
  return true;
}

} // namespace packml_sm
