// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---
// pybind11 wrapper for TransitionGuard, HeartbeatState, PackmlNodeProtocol and PackML enums.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <packml_ros/transition_guard.hpp>
#include <packml_sm/common.hpp>

namespace py = pybind11;

PYBIND11_MODULE(_packml_bindings, m)
{
  m.doc() = "PackML ROS2 C++ bindings: TransitionGuard, HeartbeatState, PackmlNodeProtocol and enums";

  // --- Enums ---

  py::enum_<packml_sm::State>(m, "State")
    .value("UNDEFINED", packml_sm::State::UNDEFINED)
    .value("CLEARING", packml_sm::State::CLEARING)
    .value("STOPPED", packml_sm::State::STOPPED)
    .value("STARTING", packml_sm::State::STARTING)
    .value("IDLE", packml_sm::State::IDLE)
    .value("SUSPENDED", packml_sm::State::SUSPENDED)
    .value("EXECUTE", packml_sm::State::EXECUTE)
    .value("STOPPING", packml_sm::State::STOPPING)
    .value("ABORTING", packml_sm::State::ABORTING)
    .value("ABORTED", packml_sm::State::ABORTED)
    .value("HOLDING", packml_sm::State::HOLDING)
    .value("HELD", packml_sm::State::HELD)
    .value("UNHOLDING", packml_sm::State::UNHOLDING)
    .value("SUSPENDING", packml_sm::State::SUSPENDING)
    .value("UNSUSPENDING", packml_sm::State::UNSUSPENDING)
    .value("RESETTING", packml_sm::State::RESETTING)
    .value("COMPLETING", packml_sm::State::COMPLETING)
    .value("COMPLETE", packml_sm::State::COMPLETE);

  py::enum_<packml_sm::TransitionCmd>(m, "TransitionCmd")
    .value("NO_COMMAND", packml_sm::TransitionCmd::NO_COMMAND)
    .value("RESET", packml_sm::TransitionCmd::RESET)
    .value("START", packml_sm::TransitionCmd::START)
    .value("STOP", packml_sm::TransitionCmd::STOP)
    .value("HOLD", packml_sm::TransitionCmd::HOLD)
    .value("UNHOLD", packml_sm::TransitionCmd::UNHOLD)
    .value("SUSPEND", packml_sm::TransitionCmd::SUSPEND)
    .value("UNSUSPEND", packml_sm::TransitionCmd::UNSUSPEND)
    .value("ABORT", packml_sm::TransitionCmd::ABORT)
    .value("CLEAR", packml_sm::TransitionCmd::CLEAR);

  // --- TransitionResult ---

  py::class_<packml_ros::TransitionResult>(m, "TransitionResult")
    .def(py::init<>())
    .def_readwrite("accepted", &packml_ros::TransitionResult::accepted)
    .def_readwrite("already_there", &packml_ros::TransitionResult::already_there)
    .def_readwrite("error", &packml_ros::TransitionResult::error)
    .def("__repr__", [](const packml_ros::TransitionResult & r) {
      return "<TransitionResult accepted=" + std::to_string(r.accepted) +
             " already_there=" + std::to_string(r.already_there) +
             " error='" + r.error + "'>";
    });

  // --- TransitionGuard ---

  py::class_<packml_ros::TransitionGuard>(m, "TransitionGuard")
    .def(py::init<>())
    .def("request_state", &packml_ros::TransitionGuard::request_state,
         py::arg("target"), "Request a state transition. Returns TransitionResult.")
    .def("request_mode", &packml_ros::TransitionGuard::request_mode,
         py::arg("target"), "Request a mode transition. Returns TransitionResult.")
    .def("on_status_update", &packml_ros::TransitionGuard::on_status_update,
         py::arg("state"), py::arg("mode"),
         "Update internal state from manager status. Returns True if changed.")
    .def_property_readonly("current_state", &packml_ros::TransitionGuard::current_state)
    .def_property_readonly("current_mode", &packml_ros::TransitionGuard::current_mode)
    .def_property_readonly("is_switching_state", &packml_ros::TransitionGuard::is_switching_state)
    .def_property_readonly("is_switching_mode", &packml_ros::TransitionGuard::is_switching_mode);

  // --- HeartbeatState::LatchSnapshot ---
  // Torn-read-free snapshot of the event latch (one lock); the only way to read it.

  py::class_<packml_ros::HeartbeatState::LatchSnapshot>(m, "LatchSnapshot")
    .def_readonly("active", &packml_ros::HeartbeatState::LatchSnapshot::active)
    .def_readonly("status", &packml_ros::HeartbeatState::LatchSnapshot::status)
    .def_readonly("action", &packml_ros::HeartbeatState::LatchSnapshot::action)
    .def_readonly("error_code", &packml_ros::HeartbeatState::LatchSnapshot::error_code)
    .def_readonly("message", &packml_ros::HeartbeatState::LatchSnapshot::message);

  // --- HeartbeatState ---
  // Not directly constructible from Python; always accessed via PackmlNodeProtocol.heartbeat.

  py::class_<packml_ros::HeartbeatState>(m, "HeartbeatState")
    .def("init", &packml_ros::HeartbeatState::init,
         py::arg("node_name"), py::arg("interval_ms"),
         "Initialise heartbeat state: node name and publishing interval.")
    .def("next_sequence", &packml_ros::HeartbeatState::next_sequence,
         "Increment and return the next sequence number.")
    .def_property_readonly("interval_ms", &packml_ros::HeartbeatState::interval_ms)
    .def_property_readonly("node_name", &packml_ros::HeartbeatState::node_name)
    .def("set_active", &packml_ros::HeartbeatState::set_active, py::arg("active"),
         "Pause or resume heartbeat publishing.")
    .def_property_readonly("is_active", &packml_ros::HeartbeatState::is_active)
    .def("set_latch", &packml_ros::HeartbeatState::set_latch,
         py::arg("status"), py::arg("action"), py::arg("error_code"), py::arg("message"),
         "Latch a sticky health state for the periodic publisher to repeat.")
    .def("clear_latch", &packml_ros::HeartbeatState::clear_latch,
         "Clear the latched health; resume get_health_status()-driven publishing.")
    .def("latch_snapshot", &packml_ros::HeartbeatState::latch_snapshot,
         "Torn-read-free snapshot of the whole latch (active/status/action/error_code/"
         "message) under one lock — use instead of per-field reads.");

  // --- PackmlNodeProtocol ---

  py::class_<packml_ros::PackmlNodeProtocol>(m, "PackmlNodeProtocol")
    .def(py::init<>())
    .def_property_readonly("transitions",
      [](packml_ros::PackmlNodeProtocol & p) -> packml_ros::TransitionGuard & {
        return p.transitions; },
      py::return_value_policy::reference_internal)
    .def_property_readonly("heartbeat",
      [](packml_ros::PackmlNodeProtocol & p) -> packml_ros::HeartbeatState & {
        return p.heartbeat; },
      py::return_value_policy::reference_internal);

  // --- Topic / Service names ---

  m.attr("STATE_TRANSITION_SERVICE") = packml_ros::kStateTransitionService;
  m.attr("MODE_TRANSITION_SERVICE") = packml_ros::kModeTransitionService;
  m.attr("STATUS_TOPIC") = packml_ros::kStatusTopic;
  m.attr("ALARMS_TOPIC") = packml_ros::kAlarmsTopic;
  m.attr("HEARTBEAT_TOPIC") = packml_ros::kHeartbeatTopic;
}
