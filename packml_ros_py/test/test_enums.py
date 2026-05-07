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

"""Unit tests for PackML Python enums (backed by pybind11 C++ enums)."""

from packml_ros_py.enums import State, TransitionCmd


class TestStateEnum:
    """Verify State enum values match C++ packml_sm::State."""

    def test_state_values(self):
        assert int(State.UNDEFINED) == 0
        assert int(State.CLEARING) == 1
        assert int(State.STOPPED) == 2
        assert int(State.STARTING) == 3
        assert int(State.IDLE) == 4
        assert int(State.SUSPENDED) == 5
        assert int(State.EXECUTE) == 6
        assert int(State.STOPPING) == 7
        assert int(State.ABORTING) == 8
        assert int(State.ABORTED) == 9
        assert int(State.HOLDING) == 10
        assert int(State.HELD) == 11
        assert int(State.UNHOLDING) == 12
        assert int(State.SUSPENDING) == 13
        assert int(State.UNSUSPENDING) == 14
        assert int(State.RESETTING) == 15
        assert int(State.COMPLETING) == 16
        assert int(State.COMPLETE) == 17

    def test_state_count(self):
        """All 18 PackML states must be represented."""
        assert len(State.__members__) == 18

    def test_state_names(self):
        assert State(6).name == 'EXECUTE'
        assert State(2).name == 'STOPPED'

    def test_state_from_int(self):
        assert State(4) == State.IDLE


class TestTransitionCmdEnum:
    """Verify TransitionCmd enum values match C++ packml_sm::TransitionCmd."""

    def test_cmd_values(self):
        assert int(TransitionCmd.NO_COMMAND) == 0
        assert int(TransitionCmd.RESET) == 1
        assert int(TransitionCmd.START) == 2
        assert int(TransitionCmd.STOP) == 3
        assert int(TransitionCmd.HOLD) == 4
        assert int(TransitionCmd.UNHOLD) == 5
        assert int(TransitionCmd.SUSPEND) == 6
        assert int(TransitionCmd.UNSUSPEND) == 7
        assert int(TransitionCmd.ABORT) == 8
        assert int(TransitionCmd.CLEAR) == 9

    def test_cmd_count(self):
        assert len(TransitionCmd.__members__) == 10
