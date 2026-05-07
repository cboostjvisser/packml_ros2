# Copyright (c) 2026 PackML ROS2 Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for the C++ TransitionGuard pybind11 bindings."""

import pytest
from packml_ros_py._packml_bindings import (
    TransitionGuard,
    TransitionResult,
    State,
    TransitionCmd,
)


class TestBindingsEnums:
    """Verify C++ enums are properly exposed."""

    def test_state_values(self):
        assert int(State.UNDEFINED) == 0
        assert int(State.STOPPED) == 2
        assert int(State.EXECUTE) == 6
        assert int(State.COMPLETE) == 17

    def test_transition_cmd_values(self):
        assert int(TransitionCmd.NO_COMMAND) == 0
        assert int(TransitionCmd.RESET) == 1
        assert int(TransitionCmd.START) == 2
        assert int(TransitionCmd.CLEAR) == 9


class TestTransitionGuardBindings:
    """Verify TransitionGuard works through pybind11."""

    def test_initial_state(self):
        guard = TransitionGuard()
        assert guard.current_state == State.UNDEFINED
        assert guard.current_mode == 0
        assert guard.is_switching_state is False
        assert guard.is_switching_mode is False

    def test_request_state_accepted(self):
        guard = TransitionGuard()
        result = guard.request_state(State.STOPPED)
        assert result.accepted is True
        assert result.already_there is False
        assert result.error == ''

    def test_request_state_already_there(self):
        guard = TransitionGuard()
        guard.on_status_update(State.IDLE, 0)
        result = guard.request_state(State.IDLE)
        assert result.accepted is True
        assert result.already_there is True

    def test_request_mode_accepted(self):
        guard = TransitionGuard()
        result = guard.request_mode(1)
        assert result.accepted is True
        assert result.already_there is False

    def test_status_update_changes_state(self):
        guard = TransitionGuard()
        changed = guard.on_status_update(State.EXECUTE, 2)
        assert changed is True
        assert guard.current_state == State.EXECUTE
        assert guard.current_mode == 2

    def test_status_update_no_change(self):
        guard = TransitionGuard()
        guard.on_status_update(State.IDLE, 1)
        changed = guard.on_status_update(State.IDLE, 1)
        assert changed is False

    def test_switching_state_flag(self):
        guard = TransitionGuard()
        guard.request_state(State.STOPPED)
        assert guard.is_switching_state is True
        guard.on_status_update(State.STOPPED, 0)
        assert guard.is_switching_state is False

    def test_warn_double_state_switch(self):
        guard = TransitionGuard()
        guard.request_state(State.STOPPED)
        result = guard.request_state(State.IDLE)
        assert result.accepted is True
        assert result.error != ''

    def test_result_repr(self):
        result = TransitionResult()
        assert 'TransitionResult' in repr(result)
