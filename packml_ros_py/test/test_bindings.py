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

"""Tests for the C++ PackmlNodeProtocol pybind11 bindings."""

import pytest
from packml_ros_py._packml_bindings import (
    TransitionGuard,
    HeartbeatState,
    PackmlNodeProtocol,
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
        tc = TransitionGuard()
        assert tc.current_state == State.UNDEFINED
        assert tc.current_mode == 0
        assert tc.is_switching_state is False
        assert tc.is_switching_mode is False

    def test_request_state_accepted(self):
        tc = TransitionGuard()
        claimed = tc.request_state(State.STOPPED)
        assert claimed.result.accepted is True
        assert claimed.result.already_there is False
        assert claimed.result.error == ''
        assert claimed.arm.armed is True

    def test_arm_is_released_when_the_request_that_took_it_ends(self):
        # The arm is scope-bound: dropping the ArmedTransition ends the request, so the next
        # goal is accepted. Before this it was released only by a status reporting a DIFFERENT
        # state, which stranded it whenever the state never changed.
        tc = TransitionGuard()
        tc.on_status_update(State.STOPPED, 0)
        claimed = tc.request_state(State.RESETTING)
        assert tc.is_switching_state is True
        claimed.arm.release()
        assert tc.is_switching_state is False
        assert tc.request_state(State.IDLE).result.accepted is True

    def test_arm_is_a_context_manager(self):
        tc = TransitionGuard()
        tc.on_status_update(State.STOPPED, 0)
        claimed = tc.request_state(State.RESETTING)
        with claimed.arm:
            assert tc.is_switching_state is True
        assert tc.is_switching_state is False

    def test_admitted_goal_not_shortcut_by_racing_status_echo(self):
        # already_there is decided at admission, where current_state has not yet been
        # overwritten by this very goal's own status echo.
        tc = TransitionGuard()
        tc.admit_state(State.IDLE)
        tc.on_status_update(State.IDLE, 0)
        claimed = tc.claim_state(State.IDLE)
        assert claimed.result.accepted is True
        assert claimed.result.already_there is False

    def test_request_state_already_there(self):
        tc = TransitionGuard()
        tc.on_status_update(State.IDLE, 0)
        claimed = tc.request_state(State.IDLE)
        assert claimed.result.accepted is True
        assert claimed.result.already_there is True
        assert claimed.arm.armed is False

    def test_request_mode_accepted(self):
        tc = TransitionGuard()
        result = tc.request_mode(1)
        assert result.accepted is True
        assert result.already_there is False

    def test_status_update_changes_state(self):
        tc = TransitionGuard()
        changed = tc.on_status_update(State.EXECUTE, 2)
        assert changed is True
        assert tc.current_state == State.EXECUTE
        assert tc.current_mode == 2

    def test_status_update_no_change(self):
        tc = TransitionGuard()
        tc.on_status_update(State.IDLE, 1)
        changed = tc.on_status_update(State.IDLE, 1)
        assert changed is False

    def test_switching_state_flag(self):
        tc = TransitionGuard()
        claimed = tc.request_state(State.STOPPED)
        assert tc.is_switching_state is True
        # A status confirming the arm's own target settles the transition and frees the node.
        tc.on_status_update(State.STOPPED, 0)
        assert tc.is_switching_state is False
        del claimed

    def test_reject_different_state_while_switching(self):
        # While a state transition is in flight, a request for a DIFFERENT state must
        # be rejected (not silently override the in-flight one). Expected: accepted is
        # False with a reason, and the original target survives to completion.
        tc = TransitionGuard()
        stopped = tc.request_state(State.STOPPED)
        assert stopped.result.accepted is True
        result = tc.request_state(State.IDLE).result
        assert result.accepted is False
        assert result.error != ''
        # Original target preserved: a status update to STOPPED still completes it.
        assert tc.is_switching_state is True
        tc.on_status_update(State.STOPPED, 0)
        assert tc.is_switching_state is False
        assert tc.current_state == State.STOPPED

class TestPackmlNodeProtocolComposition:
    """Verify PackmlNodeProtocol exposes all three concerns as named sub-objects."""

    def test_has_transitions_and_heartbeat(self):
        p = PackmlNodeProtocol()
        assert hasattr(p, 'transitions')
        assert hasattr(p, 'heartbeat')

    def test_transitions_is_transition_guard(self):
        p = PackmlNodeProtocol()
        assert p.transitions.current_state == State.UNDEFINED

    def test_transitions_request_state(self):
        p = PackmlNodeProtocol()
        claimed = p.transitions.request_state(State.STOPPED)
        assert claimed.result.accepted is True

    def test_heartbeat_init_and_properties(self):
        p = PackmlNodeProtocol()
        p.heartbeat.init('test_node', 500)
        assert p.heartbeat.node_name == 'test_node'
        assert p.heartbeat.interval_ms == 500

    def test_heartbeat_sequence_increments(self):
        p = PackmlNodeProtocol()
        assert p.heartbeat.next_sequence() == 1
        assert p.heartbeat.next_sequence() == 2

    def test_heartbeat_active_default(self):
        p = PackmlNodeProtocol()
        assert p.heartbeat.is_active is True

    def test_heartbeat_set_active_false(self):
        p = PackmlNodeProtocol()
        p.heartbeat.set_active(False)
        assert p.heartbeat.is_active is False


class TestHeartbeatProtocol:
    """Verify HeartbeatState accessed via PackmlNodeProtocol."""

    def test_init_heartbeat(self):
        p = PackmlNodeProtocol()
        p.heartbeat.init('my_node', 500)
        assert p.heartbeat.node_name == 'my_node'
        assert p.heartbeat.interval_ms == 500

    def test_sequence_increments(self):
        p = PackmlNodeProtocol()
        assert p.heartbeat.next_sequence() == 1
        assert p.heartbeat.next_sequence() == 2
        assert p.heartbeat.next_sequence() == 3

    def test_heartbeat_active_default(self):
        p = PackmlNodeProtocol()
        assert p.heartbeat.is_active is True

    def test_set_heartbeat_active_false(self):
        p = PackmlNodeProtocol()
        p.heartbeat.set_active(False)
        assert p.heartbeat.is_active is False

    def test_set_heartbeat_active_resume(self):
        p = PackmlNodeProtocol()
        p.heartbeat.set_active(False)
        p.heartbeat.set_active(True)
        assert p.heartbeat.is_active is True
