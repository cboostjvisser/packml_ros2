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
#include "packml_sm/transitions/sc_transition.hpp"
#include "packml_sm/events/sc_event.hpp"
#include "packml_sm/states/state.hpp"
#include "packml_sm/transitions/packml_transitions.hpp"

namespace packml_sm {

StateCompleteTransition::StateCompleteTransition(PackmlState &from,
                                                 PackmlState &to) {
  this->setTargetState(&to);
  from.addTransition(this);
  PACKML_INFO_STREAM("packml_sm", "Creating state complete transition from "
            << from.name() << " to " << to.name());
}

bool StateCompleteTransition::eventTest(QEvent *e) {
  if (e->type() != QEvent::Type(PACKML_STATE_COMPLETE_EVENT_TYPE)) {
    return false;
  }

  // Deliberately does NOT consult the mode mask. A mode governs what an operator may COMMAND
  // (see target_available_in_current_mode() in packml_transitions.hpp); it must never decide
  // whether the machine may FINISH a state it is already executing. Calling the base's
  // eventTest() from here would apply that check, so masking a state would strand the machine
  // inside the acting state that leads to it, with no exit, no alarm and no timeout, while the
  // command that got it there still reported success.
  //
  // It does check WHOSE completion this is. Matching on event type alone made every completion
  // interchangeable, so one state's could be consumed as another's, or as a later visit's.
  return completion_matches_this_activation(static_cast<StateCompleteEvent *>(e)->token);
}

} // namespace packml_sm
