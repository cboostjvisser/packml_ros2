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
#include "packml_sm/transitions/error_transition.hpp"
#include "packml_sm/events/error_event.hpp"

namespace packml_sm {

ErrorTransition::ErrorTransition(PackmlState &from, PackmlState &to) {
  this->setTargetState(&to);
  from.addTransition(this);
  PACKML_INFO_STREAM("packml_sm", "Creating error transition from " << from.name()
            << " to " << to.name());
}

bool ErrorTransition::eventTest(QEvent *e) {
  if (e->type() != QEvent::Type(PACKML_ERROR_EVENT_TYPE)) {
    return false;
  }

  // Deliberately does NOT consult the mode mask -- and this one is a safety property, not just
  // a correctness one. A fault must always be able to escalate; a mode that could suppress the
  // error path would mean configuration deciding whether the machine is allowed to react to a
  // fault at all. Calling the base's eventTest() from here would apply that check, so masking
  // ABORTING would silently disarm error escalation from every abortable state.
  //
  // The same principle sets how much of the event's identity is checked: only enough to keep the
  // fault from being blamed on a state that did not raise it. NOT whether the fault is still
  // current -- a late fault is a real fault, and it escalates through the nearest enclosing
  // superstate instead. Contrast StateCompleteTransition, which discards a stale completion,
  // because acting on a completion that has been overtaken means skipping real work.
  return fault_may_be_attributed_here(static_cast<ErrorEvent *>(e)->token);
}

} // namespace packml_sm
