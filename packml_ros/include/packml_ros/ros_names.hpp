// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---
// Canonical ROS interface names for the PackML system — the single home for every
// topic, service, and parameter name shared between the manager, the C++
// PackmlNodeInterface, the Python PackmlNode (exported via pybind11 in
// packml_ros_py/src/bindings.cpp), and the tests. Add new names here, not inline.

#pragma once

namespace packml_ros {

// Canonical topic / service names used by both C++ PackmlNodeInterface and Python PackmlNode.
static constexpr auto kStateTransitionService = "packml_state_transition";
static constexpr auto kModeTransitionService  = "packml_mode_transition";
static constexpr auto kStatusTopic            = "packml_status";
static constexpr auto kAlarmsTopic            = "packml_alarms";

// Manager node-relative service / topic names (created/used under the node's "~/" namespace).
static constexpr auto kChangeModeService  = "changeMode";
static constexpr auto kChangeStateService = "changeState";
static constexpr auto kAllStatusService   = "allStatus";
static constexpr auto kHeartbeatTopic     = "heartbeat";  // EM publishes "~/heartbeat"; manager subscribes "/<node>/heartbeat"

// Canonical ROS parameter names declared by the manager / node interface.
static constexpr auto kParamHeartbeatIntervalMs          = "heartbeat_interval_ms";
static constexpr auto kParamNodeNames                    = "node_names";
static constexpr auto kParamRequiredNodes                = "required_nodes";
static constexpr auto kParamHeartbeatTimeoutFactor       = "heartbeat_timeout_factor";
static constexpr auto kParamManualModeAllowsHealthBypass = "manual_mode_allows_health_bypass";
static constexpr auto kParamManualMode                   = "manual_mode";
static constexpr auto kParamHeartbeatStartupGraceMs      = "heartbeat_startup_grace_ms";
static constexpr auto kParamInitialMode                  = "initial_mode";
static constexpr auto kParamModesConfigFile              = "modes_config_file";
static constexpr auto kParamErrorCatalogFile              = "error_catalog_file";
static constexpr auto kParamLanguage                      = "language";

}  // namespace packml_ros
