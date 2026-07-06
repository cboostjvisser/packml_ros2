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

import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from packml_msgs.srv import StateTransition, ModeTransition
from packml_msgs.msg import Status, State as StateMsg, Mode as ModeMsg, NodeHealth, NodeHeartbeat

from packml_ros_py import PackmlNode, State
from packml_ros_py import (
    STATE_TRANSITION_SERVICE, MODE_TRANSITION_SERVICE, STATUS_TOPIC, HEARTBEAT_TOPIC,
    PARAM_HEARTBEAT_INTERVAL_MS,
)
import packml_modes

# Latched status QoS — matches PackmlNode's status subscription (RELIABLE +
# TRANSIENT_LOCAL). A best-effort/volatile publisher would be QoS-incompatible
# with that subscription and never deliver.
_STATUS_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

# Heartbeat QoS — matches PackmlNode's heartbeat publisher (BEST_EFFORT, depth 5).
_HEARTBEAT_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)


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


@pytest.fixture
def fast_packml_node():
    """PackmlNode with a short heartbeat interval so periodic-publish tests run quickly."""
    node = PackmlNode(
        'test_packml_node_fast',
        parameter_overrides=[Parameter(PARAM_HEARTBEAT_INTERVAL_MS, value=50)],
    )
    yield node
    node.destroy_node()


@pytest.fixture
def fast_executor(fast_packml_node, client_node):
    """Create an executor with the fast-heartbeat node and the client helper node."""
    exec_ = SingleThreadedExecutor()
    exec_.add_node(fast_packml_node)
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
        packml_node._protocol.transitions.on_status_update(State.IDLE, 0)

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
        packml_node._protocol.transitions.on_status_update(State.UNDEFINED, packml_modes.MAINTENANCE)

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
        pub = client_node.create_publisher(Status, STATUS_TOPIC, _STATUS_QOS)
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

        pub = client_node.create_publisher(Status, STATUS_TOPIC, _STATUS_QOS)
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

        packml_node._protocol.transitions.on_status_update(State.IDLE, packml_modes.PRODUCTION)

        pub = client_node.create_publisher(Status, STATUS_TOPIC, _STATUS_QOS)
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


class TestHeartbeatPublishing:
    """Test the periodic NodeHeartbeat publisher."""

    def test_heartbeat_published_periodically(self, fast_packml_node, client_node, fast_executor):
        """The node publishes NodeHeartbeat on its own topic at heartbeat_interval_ms.
        Expected: at least one heartbeat is received, carrying the node's own name and
        the default get_health_status() result (HEALTHY / NONE)."""
        received = []
        client_node.create_subscription(
            NodeHeartbeat, f'/test_packml_node_fast/{HEARTBEAT_TOPIC}',
            received.append, _HEARTBEAT_QOS)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and len(received) < 1:
            fast_executor.spin_once(timeout_sec=0.1)

        assert len(received) >= 1
        assert received[0].node_name == 'test_packml_node_fast'
        assert received[0].health.status == NodeHealth.HEALTHY
        assert received[0].health.action == NodeHealth.NONE

    def test_heartbeat_sequence_increments(self, fast_packml_node, client_node, fast_executor):
        """Successive heartbeats carry a strictly increasing sequence_number.
        Expected: at least 3 heartbeats received, each with a sequence_number greater
        than the previous one."""
        received = []
        client_node.create_subscription(
            NodeHeartbeat, f'/test_packml_node_fast/{HEARTBEAT_TOPIC}',
            received.append, _HEARTBEAT_QOS)

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and len(received) < 3:
            fast_executor.spin_once(timeout_sec=0.1)

        assert len(received) >= 3
        for i in range(1, len(received)):
            assert received[i].sequence_number > received[i - 1].sequence_number


class TestPostEvent:
    """Test post_event() immediate publish and latch behavior."""

    def test_post_event_publishes_immediately(self, packml_node, client_node, executor):
        """post_event() publishes a heartbeat right away, without waiting for the
        periodic timer (default interval 1000ms). Expected: the published health
        matches the posted fault, not the default get_health_status()."""
        received = []
        client_node.create_subscription(
            NodeHeartbeat, f'/test_packml_node/{HEARTBEAT_TOPIC}',
            received.append, _HEARTBEAT_QOS)
        executor.spin_once(timeout_sec=0.1)

        fault = NodeHealth()
        fault.status = NodeHealth.ERROR
        fault.action = NodeHealth.ABORT
        fault.error_code = 42
        fault.message = 'test fault'
        packml_node.post_event(fault)

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not received:
            executor.spin_once(timeout_sec=0.1)

        assert len(received) >= 1
        assert received[0].health.status == NodeHealth.ERROR
        assert received[0].health.action == NodeHealth.ABORT
        assert received[0].health.error_code == 42

    def test_post_event_latches_and_repeats(self, fast_packml_node, client_node, fast_executor):
        """After post_event() with an actionable fault, the periodic publisher must
        repeat the LATCHED health on every subsequent tick instead of calling
        get_health_status() again. Expected: several heartbeats published AFTER the
        fault all carry the latched health (a periodic tick that already fired before
        the fault would correctly show healthy, so it must not be counted here)."""
        received = []
        client_node.create_subscription(
            NodeHeartbeat, f'/test_packml_node_fast/{HEARTBEAT_TOPIC}',
            received.append, _HEARTBEAT_QOS)

        # Let any already-in-flight periodic tick be delivered and recorded as the
        # watermark, so it can't be mistaken for a post-fault heartbeat below.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not received:
            fast_executor.spin_once(timeout_sec=0.1)
        watermark = received[-1].sequence_number if received else 0

        fault = NodeHealth()
        fault.status = NodeHealth.ERROR
        fault.action = NodeHealth.HOLD
        fault.error_code = 7
        fast_packml_node.post_event(fault)

        deadline = time.monotonic() + 2.0
        while (time.monotonic() < deadline and
               sum(1 for m in received if m.sequence_number > watermark) < 3):
            fast_executor.spin_once(timeout_sec=0.1)

        after_fault = [m for m in received if m.sequence_number > watermark]
        assert len(after_fault) >= 3
        for msg in after_fault:
            assert msg.health.action == NodeHealth.HOLD
            assert msg.health.error_code == 7

    def test_post_event_clear_resumes_getter(self, client_node):
        """Posting a healthy/NONE event clears the latch, so subsequent heartbeats
        resume calling get_health_status() instead of repeating the old fault.
        Expected: after the clear, published heartbeats reflect the node's overridden
        get_health_status() (DEGRADED/WARN), not the previously-latched fault."""
        class OverriddenNode(PackmlNode):
            def __init__(self):
                super().__init__(
                    'test_packml_node_override',
                    parameter_overrides=[Parameter(PARAM_HEARTBEAT_INTERVAL_MS, value=50)])

            def get_health_status(self):
                h = NodeHealth()
                h.status = NodeHealth.DEGRADED
                h.action = NodeHealth.WARN
                h.message = 'overridden'
                return h

        node = OverriddenNode()
        exec_ = SingleThreadedExecutor()
        exec_.add_node(node)
        exec_.add_node(client_node)

        try:
            received = []
            client_node.create_subscription(
                NodeHeartbeat, f'/test_packml_node_override/{HEARTBEAT_TOPIC}',
                received.append, _HEARTBEAT_QOS)

            fault = NodeHealth()
            fault.status = NodeHealth.ERROR
            fault.action = NodeHealth.ABORT
            node.post_event(fault)

            # Wait until the latched fault itself has been observed before clearing
            # it, then use its sequence_number as the watermark for "after the clear".
            deadline = time.monotonic() + 1.0
            while (time.monotonic() < deadline and
                   not any(m.health.action == NodeHealth.ABORT for m in received)):
                exec_.spin_once(timeout_sec=0.1)
            assert any(m.health.action == NodeHealth.ABORT for m in received), (
                "latched fault heartbeat was never observed")
            watermark = received[-1].sequence_number

            clear = NodeHealth()
            clear.action = NodeHealth.NONE
            node.post_event(clear)

            deadline = time.monotonic() + 2.0
            while (time.monotonic() < deadline and
                   sum(1 for m in received if m.sequence_number > watermark) < 3):
                exec_.spin_once(timeout_sec=0.1)

            # The clear event's own immediate publish (action=NONE) isn't a periodic
            # tick; check only the PERIODIC heartbeats published after it.
            after_clear = [m for m in received if m.sequence_number > watermark]
            periodic_after_clear = [m for m in after_clear if m.health.action != NodeHealth.NONE]
            assert len(periodic_after_clear) >= 2
            for msg in periodic_after_clear:
                assert msg.health.action == NodeHealth.WARN
                assert msg.health.status == NodeHealth.DEGRADED
        finally:
            exec_.shutdown()
            node.destroy_node()
