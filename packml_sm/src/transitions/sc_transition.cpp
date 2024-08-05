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
#include "packml_sm/transitions/sc_transition.hpp"
#include "packml_sm/events/sc_event.hpp"

namespace packml_sm {

StateCompleteTransition::StateCompleteTransition(PackmlState &from,
                                                 PackmlState &to) {
  this->setTargetState(&to);
  from.addTransition(this);
  std::cout << "Creating state complete transition from "
            << from.name() << " to " << to.name()
            << std::endl;
}

bool StateCompleteTransition::eventTest(QEvent *e) {
  if (e->type() != QEvent::Type(PACKML_STATE_COMPLETE_EVENT_TYPE)) {
    return false;
  }
  return true;
}

} // namespace packml_sm
