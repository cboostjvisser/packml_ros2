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
#include "packml_sm/events/activation_token.hpp"

namespace packml_sm {

static int PACKML_STATE_COMPLETE_EVENT_TYPE = QEvent::User + 2;

struct StateCompleteEvent : public QEvent {
  // `token` names the state and the visit whose work this event reports finished. A completion is
  // a claim about one specific activation, so StateCompleteTransition requires it: an event
  // arriving without one cannot be attributed and is refused rather than credited to whichever
  // state happens to be active.
  explicit StateCompleteEvent(const ActivationToken & token_value = {})
      : QEvent(QEvent::Type(PACKML_STATE_COMPLETE_EVENT_TYPE)), token(token_value) {}

  ActivationToken token;
};
} // namespace packml_sm