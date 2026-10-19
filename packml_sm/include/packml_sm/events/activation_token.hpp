// Copyright (c) 2026 PackML ROS2 Contributors
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

#include <cstdint>

namespace packml_sm {

struct PackmlState;

// Which state, in which of its activations, produced an event.
//
// Anonymous completion and error events are not enough: with any StateCompleteEvent satisfying any
// armed StateCompleteTransition, an event minted by one state is consumed as another state's
// completion, or as a later completion of the same state. Both are reachable, because a bound
// operation runs on a worker thread and its event is queued: the machine can leave the state, or
// leave and re-enter it, before Qt gets round to processing what that state posted.
//
// `activation` counts entries and exits of `origin` (see PackmlState::activation()), so it names
// one specific visit rather than the state in general. Comparing it is what distinguishes "this
// state's completion" from "a completion this state posted two visits ago".
struct ActivationToken
{
  const PackmlState * origin{nullptr};
  uint64_t activation{0};
};

}  // namespace packml_sm
