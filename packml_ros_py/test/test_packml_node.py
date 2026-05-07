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

"""Unit tests for PackmlNode class (ROS2 node-level tests)."""

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor

from packml_msgs.srv import StateTransition, ModeTransition
from packml_msgs.msg import Status, State as StateMsg, Mode as ModeMsg

from packml_ros_py import PackmlNode, State
from packml_ros_py import STATE_TRANSITION_SERVICE, MODE_TRANSITION_SERVICE, STATUS_TOPIC
import packml_modes


@pytest.fixture(scope='module', autouse=True)
def rclpy_init_shutdown():
    """Initialize and shutdown rclpy for the test module."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def packml_node():
    """Create a PackmlNode instance for testing."""
    node = PackmlNode('test_packml_node')
    yield node
    node.destroy_node()


@pytest.fixture
def client_node():
    """Create a helper node for calling services."""
    node = rclpy.create_node('test_client_node')
    yield node
    node.destroy_node()


@pytest.fixture
def executor(packml_node, client_node):
    """Create an executor with both nodes."""
    exec_ = SingleThreadedExecutor()
    exec_.add_node(packml_node)
    exec_.add_node(client_node)
    yield exec_
    exec_.shutdown()


class TestPackmlNodeInit:
    """Test PackmlNode initialization."""

    def test_initial_state_is_undefined(self, packml_node):
        assert packml_node.current_state == State.UNDEFINED

    def test_initial_mode_is_zero(self, packml_node):
        assert packml_node.current_mode == 0

    def test_services_are_created(self, packml_node, client_node, executor):
        state_client = client_node.create_client(
            StateTransition,
            f'/test_packml_node/{STATE_TRANSITION_SERVICE}'
        )
        mode_client = client_node.create_client(
            ModeTransition,
            f'/test_packml_node/{MODE_TRANSITION_SERVICE}'
        )
        # Spin briefly to allow discovery
        executor.spin_once(timeout_sec=0.1)

        assert state_client.wait_for_service(timeout_sec=2.0)
        assert mode_client.wait_for_service(timeout_sec=2.0)


class TestStateTransitionService:
    """Test the ~/packml_state_transition service handling."""

    def test_accept_state_transition(self, packml_node, client_node, executor):
        """Default node accepts all transitions."""
        state_client = client_node.create_client(
            StateTransition,
            f'/test_packml_node/{STATE_TRANSITION_SERVICE}'
        )
        executor.spin_once(timeout_sec=0.1)
        assert state_client.wait_for_service(timeout_sec=2.0)

        req = StateTransition.Request()
        req.state.val = int(State.STOPPED)

        future = state_client.call_async(req)
        # Spin until response
        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        result = future.result()
        assert result.success is True

    def test_already_in_state(self, packml_node, client_node, executor):
        """If node is already in the requested state, return success immediately."""
        # Set state to IDLE via the guard's status update
        packml_node._guard.on_status_update(State.IDLE, 0)

        state_client = client_node.create_client(
            StateTransition,
            f'/test_packml_node/{STATE_TRANSITION_SERVICE}'
        )
        executor.spin_once(timeout_sec=0.1)
        assert state_client.wait_for_service(timeout_sec=2.0)

        req = StateTransition.Request()
        req.state.val = int(State.IDLE)

        future = state_client.call_async(req)
        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        result = future.result()
        assert result.success is True

    def test_reject_state_transition(self, client_node, executor):
        """Node that overrides callback to reject transitions."""
        class RejectingNode(PackmlNode):
            def __init__(self):
                super().__init__('rejecting_node')

            def on_state_transition_request(self, target_state):
                return False

        rejecting_node = RejectingNode()
        executor.add_node(rejecting_node)

        state_client = client_node.create_client(
            StateTransition,
            f'/rejecting_node/{STATE_TRANSITION_SERVICE}'
        )
        executor.spin_once(timeout_sec=0.1)
        assert state_client.wait_for_service(timeout_sec=2.0)

        req = StateTransition.Request()
        req.state.val = int(State.EXECUTE)

        future = state_client.call_async(req)
        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        result = future.result()
        assert result.success is False
        assert 'rejected' in result.message.lower()

        rejecting_node.destroy_node()


class TestModeTransitionService:
    """Test the ~/packml_mode_transition service handling."""

    def test_accept_mode_transition(self, packml_node, client_node, executor):
        """Default node accepts all mode transitions."""
        mode_client = client_node.create_client(
            ModeTransition,
            f'/test_packml_node/{MODE_TRANSITION_SERVICE}'
        )
        executor.spin_once(timeout_sec=0.1)
        assert mode_client.wait_for_service(timeout_sec=2.0)

        req = ModeTransition.Request()
        req.mode.val = packml_modes.PRODUCTION

        future = mode_client.call_async(req)
        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        result = future.result()
        assert result.success is True

    def test_already_in_mode(self, packml_node, client_node, executor):
        """If node is already in the requested mode, return success."""
        packml_node._guard.on_status_update(State.UNDEFINED, packml_modes.MAINTENANCE)

        mode_client = client_node.create_client(
            ModeTransition,
            f'/test_packml_node/{MODE_TRANSITION_SERVICE}'
        )
        executor.spin_once(timeout_sec=0.1)
        assert mode_client.wait_for_service(timeout_sec=2.0)

        req = ModeTransition.Request()
        req.mode.val = packml_modes.MAINTENANCE

        future = mode_client.call_async(req)
        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        result = future.result()
        assert result.success is True

    def test_reject_mode_transition(self, client_node, executor):
        """Node that rejects mode transitions."""
        class ModeRejectingNode(PackmlNode):
            def __init__(self):
                super().__init__('mode_rejecting_node')

            def on_mode_transition_request(self, target_mode):
                return False

        rejecting_node = ModeRejectingNode()
        executor.add_node(rejecting_node)

        mode_client = client_node.create_client(
            ModeTransition,
            f'/mode_rejecting_node/{MODE_TRANSITION_SERVICE}'
        )
        executor.spin_once(timeout_sec=0.1)
        assert mode_client.wait_for_service(timeout_sec=2.0)

        req = ModeTransition.Request()
        req.mode.val = packml_modes.MANUAL

        future = mode_client.call_async(req)
        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        result = future.result()
        assert result.success is False
        assert 'rejected' in result.message.lower()

        rejecting_node.destroy_node()


class TestStatusSubscription:
    """Test packml_status subscription handling."""

    def test_status_updates_state(self, packml_node, client_node, executor):
        """Verify status topic updates internal state."""
        pub = client_node.create_publisher(Status, STATUS_TOPIC, 10)
        executor.spin_once(timeout_sec=0.1)

        msg = Status()
        msg.state.val = int(State.IDLE)
        msg.mode.val = packml_modes.PRODUCTION

        # Publish and spin to deliver
        pub.publish(msg)
        for _ in range(10):
            executor.spin_once(timeout_sec=0.1)

        assert packml_node.current_state == State.IDLE
        assert packml_node.current_mode == packml_modes.PRODUCTION

    def test_status_callback_called(self, client_node, executor):
        """Verify on_packml_status_changed is called on status update."""
        callback_data = []

        class TrackingNode(PackmlNode):
            def __init__(self):
                super().__init__('tracking_node')

            def on_packml_status_changed(self, status):
                callback_data.append(status)

        tracking_node = TrackingNode()
        executor.add_node(tracking_node)

        pub = client_node.create_publisher(Status, STATUS_TOPIC, 10)
        executor.spin_once(timeout_sec=0.1)

        msg = Status()
        msg.state.val = int(State.EXECUTE)
        msg.mode.val = packml_modes.MAINTENANCE

        pub.publish(msg)
        for _ in range(10):
            executor.spin_once(timeout_sec=0.1)

        assert len(callback_data) == 1
        assert State(callback_data[0].state.val) == State.EXECUTE

        tracking_node.destroy_node()

    def test_no_callback_when_state_unchanged(self, packml_node, client_node, executor):
        """If state/mode haven't changed, callback should not fire."""
        callback_count = [0]
        original_cb = packml_node.on_packml_status_changed
        packml_node.on_packml_status_changed = lambda s: callback_count.__setitem__(0, callback_count[0] + 1)

        packml_node._guard.on_status_update(State.IDLE, packml_modes.PRODUCTION)

        pub = client_node.create_publisher(Status, STATUS_TOPIC, 10)
        executor.spin_once(timeout_sec=0.1)

        # Publish same state
        msg = Status()
        msg.state.val = int(State.IDLE)
        msg.mode.val = packml_modes.PRODUCTION

        pub.publish(msg)
        for _ in range(10):
            executor.spin_once(timeout_sec=0.1)

        assert callback_count[0] == 0


class TestSubclassing:
    """Test that subclassing PackmlNode works as expected."""

    def test_custom_node_receives_transitions(self, client_node, executor):
        """A custom subclass gets state transition calls."""
        received_states = []

        class CustomNode(PackmlNode):
            def __init__(self):
                super().__init__('custom_node')

            def on_state_transition_request(self, target_state):
                received_states.append(target_state)
                return True

        custom_node = CustomNode()
        executor.add_node(custom_node)

        state_client = client_node.create_client(
            StateTransition,
            f'/custom_node/{STATE_TRANSITION_SERVICE}'
        )
        executor.spin_once(timeout_sec=0.1)
        assert state_client.wait_for_service(timeout_sec=2.0)

        req = StateTransition.Request()
        req.state.val = int(State.STARTING)

        future = state_client.call_async(req)
        while not future.done():
            executor.spin_once(timeout_sec=0.1)

        assert future.result().success is True
        assert received_states == [State.STARTING]

        custom_node.destroy_node()
