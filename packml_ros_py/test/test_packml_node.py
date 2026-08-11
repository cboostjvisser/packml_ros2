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
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from packml_msgs.action import StateTransition as StateTransitionAction
from packml_msgs.srv import ModeTransition
from packml_msgs.msg import Status, State as StateMsg, Mode as ModeMsg, NodeHealth, NodeHeartbeat

from packml_ros_py import PackmlNode, State
from packml_ros_py import (
    STATE_TRANSITION_ACTION, MODE_TRANSITION_SERVICE, STATUS_TOPIC, HEARTBEAT_TOPIC,
    PARAM_HEARTBEAT_INTERVAL_MS, PARAM_DEFERRED_COMPLETION_TIMEOUT_MS,
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


def _send_state_goal(action_client, executor, state_val, timeout_sec=5.0):
    """Send a ~/packml_state_transition goal and spin until its RESULT arrives (not
    merely acceptance). Returns (goal_handle, result) -- goal_handle is None if
    nothing arrived within timeout_sec; result is None if the goal was rejected
    outright at the ROS admission level or its result never arrived in time.
    """
    goal = StateTransitionAction.Goal()
    goal.state.val = state_val
    send_future = action_client.send_goal_async(goal)

    deadline = time.monotonic() + timeout_sec
    while not send_future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        return goal_handle, None

    result_future = goal_handle.get_result_async()
    while not result_future.done() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    result_response = result_future.result()
    return goal_handle, (result_response.result if result_response else None)


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
        state_action_client = ActionClient(
            client_node, StateTransitionAction, f'/test_packml_node/{STATE_TRANSITION_ACTION}')
        mode_client = client_node.create_client(
            ModeTransition,
            f'/test_packml_node/{MODE_TRANSITION_SERVICE}'
        )

        assert state_action_client.wait_for_server(timeout_sec=2.0)
        assert mode_client.wait_for_service(timeout_sec=2.0)
        state_action_client.destroy()


class TestStateTransitionAction:
    """Test the ~/packml_state_transition action handling."""

    def test_accept_state_transition(self, packml_node, client_node, executor):
        """Default node accepts all transitions and completes instantly (the default
        defers_completion() returns False for every state)."""
        action_client = ActionClient(
            client_node, StateTransitionAction, f'/test_packml_node/{STATE_TRANSITION_ACTION}')
        assert action_client.wait_for_server(timeout_sec=2.0)

        goal_handle, result = _send_state_goal(action_client, executor, int(State.STOPPED))
        assert goal_handle is not None and goal_handle.accepted
        assert result is not None
        assert result.success is True

        action_client.destroy()

    def test_already_in_state(self, packml_node, client_node, executor):
        """If node is already in the requested state, return success immediately."""
        # Set state to IDLE via the guard's status update
        packml_node._protocol.transitions.on_status_update(State.IDLE, 0)

        action_client = ActionClient(
            client_node, StateTransitionAction, f'/test_packml_node/{STATE_TRANSITION_ACTION}')
        assert action_client.wait_for_server(timeout_sec=2.0)

        goal_handle, result = _send_state_goal(action_client, executor, int(State.IDLE))
        assert goal_handle is not None and goal_handle.accepted
        assert result is not None
        assert result.success is True

        action_client.destroy()

    def test_reject_state_transition(self, client_node, executor):
        """Node that overrides callback to reject transitions."""
        class RejectingNode(PackmlNode):
            def __init__(self):
                super().__init__('rejecting_node')

            def on_state_transition_request(self, target_state):
                return False

        rejecting_node = RejectingNode()
        executor.add_node(rejecting_node)

        action_client = ActionClient(
            client_node, StateTransitionAction, f'/rejecting_node/{STATE_TRANSITION_ACTION}')
        assert action_client.wait_for_server(timeout_sec=2.0)

        goal_handle, result = _send_state_goal(action_client, executor, int(State.EXECUTE))
        assert goal_handle is not None and goal_handle.accepted
        assert result is not None
        assert result.success is False
        assert 'rejected' in result.message.lower()

        action_client.destroy()
        rejecting_node.destroy_node()

    def test_defers_completion_blocks_until_reported(self, client_node, executor):
        """A node whose defers_completion() returns True for a state does NOT
        complete that goal until that goal's own completion is reported."""
        class DeferringNode(PackmlNode):
            def __init__(self):
                super().__init__(
                    'deferring_node',
                    parameter_overrides=[Parameter(PARAM_DEFERRED_COMPLETION_TIMEOUT_MS, value=3000)])
                self.completion = None

            def defers_completion(self, state):
                return state == State.RESETTING

            def on_deferred_work(self, state, completion):
                self.completion = completion

        node = DeferringNode()
        executor.add_node(node)

        action_client = ActionClient(
            client_node, StateTransitionAction, f'/deferring_node/{STATE_TRANSITION_ACTION}')
        assert action_client.wait_for_server(timeout_sec=2.0)

        goal = StateTransitionAction.Goal()
        goal.state.val = int(State.RESETTING)
        send_future = action_client.send_goal_async(goal)
        deadline = time.monotonic() + 2.0
        while not send_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        goal_handle = send_future.result()
        assert goal_handle is not None and goal_handle.accepted

        result_future = goal_handle.get_result_async()
        # Give the goal plenty of time to (incorrectly) complete on its own -- it
        # must not, since nothing has reported its completion yet.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert not result_future.done(), 'goal completed before its completion was reported'

        assert node.completion is not None, 'the node was never asked to do deferred work'
        node.completion.report(True)

        deadline = time.monotonic() + 2.0
        while not result_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert result_future.done()
        assert result_future.result().result.success is True

        action_client.destroy()
        node.destroy_node()

    def test_defers_completion_reports_failure(self, client_node, executor):
        """An explicit failure report fails the goal, carrying the given error_code
        and message."""
        class DeferringNode(PackmlNode):
            def __init__(self):
                super().__init__(
                    'deferring_failure_node',
                    parameter_overrides=[Parameter(PARAM_DEFERRED_COMPLETION_TIMEOUT_MS, value=3000)])
                self.completion = None

            def defers_completion(self, state):
                return state == State.RESETTING

            def on_deferred_work(self, state, completion):
                self.completion = completion

        node = DeferringNode()
        executor.add_node(node)

        action_client = ActionClient(
            client_node, StateTransitionAction,
            f'/deferring_failure_node/{STATE_TRANSITION_ACTION}')
        assert action_client.wait_for_server(timeout_sec=2.0)

        goal_msg = StateTransitionAction.Goal()
        goal_msg.state.val = int(State.RESETTING)
        send_future = action_client.send_goal_async(goal_msg)
        deadline = time.monotonic() + 2.0
        while not send_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        goal_handle = send_future.result()
        assert goal_handle is not None and goal_handle.accepted

        deadline = time.monotonic() + 2.0
        while node.completion is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert node.completion is not None, 'the node was never asked to do deferred work'
        node.completion.report(success=False, error_code=7, message='simulated failure')

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + 2.0
        while not result_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert result_future.done()
        result = result_future.result().result
        assert result.success is False
        assert result.error_code == 7
        assert result.message == 'simulated failure'

        action_client.destroy()
        node.destroy_node()

    def test_defers_completion_times_out_without_report(self, client_node, executor):
        """Without a report, a deferred goal times out (bounded by
        deferred_completion_timeout_ms) and aborts rather than hanging forever."""
        class DeferringNode(PackmlNode):
            def __init__(self):
                super().__init__(
                    'deferring_timeout_node',
                    parameter_overrides=[Parameter(PARAM_DEFERRED_COMPLETION_TIMEOUT_MS, value=200)])

            def defers_completion(self, state):
                return state == State.RESETTING

            # Takes the handle and drops it on the floor, rather than leaving the base
            # implementation to report a failure straight back -- riding out the timeout is
            # the whole point of this test.
            def on_deferred_work(self, state, completion):
                pass

        node = DeferringNode()
        executor.add_node(node)

        action_client = ActionClient(
            client_node, StateTransitionAction,
            f'/deferring_timeout_node/{STATE_TRANSITION_ACTION}')
        assert action_client.wait_for_server(timeout_sec=2.0)

        goal_handle, result = _send_state_goal(
            action_client, executor, int(State.RESETTING), timeout_sec=3.0)
        assert goal_handle is not None and goal_handle.accepted
        assert result is not None
        assert result.success is False
        assert 'timed out' in result.message.lower()

        action_client.destroy()
        node.destroy_node()

    def test_per_state_deferred_completion_timeout_override(self, client_node, executor):
        """A `deferred_completion_timeout_ms.<STATE>` override takes effect for that
        state's own deferred wait instead of the (much longer) global default -- proving
        the per-state override is actually read, not just the flat timeout."""
        class DeferringNode(PackmlNode):
            def __init__(self):
                super().__init__(
                    'deferring_per_state_timeout_node',
                    parameter_overrides=[
                        Parameter(PARAM_DEFERRED_COMPLETION_TIMEOUT_MS, value=30000),
                        Parameter(f'{PARAM_DEFERRED_COMPLETION_TIMEOUT_MS}.RESETTING', value=200),
                    ])

            def defers_completion(self, state):
                return state == State.RESETTING

            # Takes the handle and drops it on the floor, rather than leaving the base
            # implementation to report a failure straight back -- riding out the timeout is
            # the whole point of this test.
            def on_deferred_work(self, state, completion):
                pass

        node = DeferringNode()
        executor.add_node(node)

        action_client = ActionClient(
            client_node, StateTransitionAction,
            f'/deferring_per_state_timeout_node/{STATE_TRANSITION_ACTION}')
        assert action_client.wait_for_server(timeout_sec=2.0)

        start = time.monotonic()
        goal_handle, result = _send_state_goal(
            action_client, executor, int(State.RESETTING), timeout_sec=3.0)
        elapsed = time.monotonic() - start
        assert goal_handle is not None and goal_handle.accepted
        assert result is not None
        assert result.success is False
        assert 'timed out' in result.message.lower()
        # The 200ms override, not the 30s global default, must be what bounded this wait.
        assert elapsed < 2.0

        action_client.destroy()
        node.destroy_node()


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

        # Destroy explicitly, now, rather than leaving it to client_node's fixture
        # teardown: STATUS_TOPIC's TRANSIENT_LOCAL durability replays a still-registered
        # publisher's last sample to whatever subscribes next, and pytest fixture teardown
        # runs late enough (after the test function returns) to let that publisher outlive
        # this test and leak a stale status message into a later one's subscriber.
        client_node.destroy_publisher(pub)

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
        # See test_status_updates_state's comment: destroy now, not at fixture teardown, so
        # this publisher's TRANSIENT_LOCAL last sample can't leak into a later test.
        client_node.destroy_publisher(pub)

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

        # See test_status_updates_state's comment: destroy now, not at fixture teardown, so
        # this publisher's TRANSIENT_LOCAL last sample can't leak into a later test.
        client_node.destroy_publisher(pub)


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

        action_client = ActionClient(
            client_node, StateTransitionAction, f'/custom_node/{STATE_TRANSITION_ACTION}')
        assert action_client.wait_for_server(timeout_sec=2.0)

        goal_handle, result = _send_state_goal(action_client, executor, int(State.STARTING))
        assert goal_handle is not None and goal_handle.accepted
        assert result is not None
        assert result.success is True
        assert received_states == [State.STARTING]

        action_client.destroy()
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
        fault.instance_id = 'cell_north'
        packml_node.post_event(fault)

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not received:
            executor.spin_once(timeout_sec=0.1)

        assert len(received) >= 1
        assert received[0].health.status == NodeHealth.ERROR
        assert received[0].health.action == NodeHealth.ABORT
        assert received[0].health.error_code == 42
        assert received[0].health.instance_id == 'cell_north'

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
        fault.instance_id = 'cell_south'
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
            assert msg.health.instance_id == 'cell_south'

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

            clear = NodeHealth()
            clear.action = NodeHealth.NONE
            node.post_event(clear)

            # The watermark is the CLEAR's own heartbeat, not whatever happened to have arrived
            # before it. post_event() publishes immediately while the periodic timer keeps ticking
            # every 50 ms, so a latched-fault tick can be published between the last message this
            # test has been handed and the clear below. Taking the watermark from received[-1] then
            # counts that PRE-clear ABORT tick as "after the clear" and fails on it -- a flake that
            # only shows up when delivery falls behind publication, i.e. under load.
            #
            # The clear's own sequence number is unambiguous because publish order IS sequence
            # order: next_sequence() and the publish happen together under the heartbeat lock (see
            # _heartbeat_lock), so nothing published before the clear can carry a higher number.
            deadline = time.monotonic() + 1.0
            while (time.monotonic() < deadline and
                   not any(m.health.action == NodeHealth.NONE for m in received)):
                exec_.spin_once(timeout_sec=0.1)
            clear_beats = [m for m in received if m.health.action == NodeHealth.NONE]
            assert clear_beats, "the clear event's own heartbeat was never observed"
            watermark = clear_beats[0].sequence_number

            deadline = time.monotonic() + 2.0
            while (time.monotonic() < deadline and
                   sum(1 for m in received if m.sequence_number > watermark) < 2):
                exec_.spin_once(timeout_sec=0.1)

            # Everything past the clear's own sequence number is a periodic tick, so these are
            # checked unfiltered -- a second NONE appearing here would be a real failure.
            periodic_after_clear = [m for m in received if m.sequence_number > watermark]
            assert len(periodic_after_clear) >= 2
            for msg in periodic_after_clear:
                assert msg.health.action == NodeHealth.WARN
                assert msg.health.status == NodeHealth.DEGRADED
        finally:
            exec_.shutdown()
            node.destroy_node()


class TestDeferredCompletionEdgeCases:
    """Edge cases for the deferred-completion handle beyond the happy-path deferred-goal
    coverage in TestStateTransitionAction."""

    def test_subclass_may_set_its_own_state_after_super_init(self):
        """A subclass whose defers_completion() reads its own attributes must survive construction.

        The base __init__ used to declare a per-state timeout parameter for every state the
        subclass defers, which meant calling the subclass's defers_completion() override 18 times
        from inside super().__init__() -- before the subclass constructor body had assigned
        anything. An override written the ordinary way, and the way this class's own docstring
        shows, then died with AttributeError while the node was being built. The C++ side cannot
        fail this way because it probes from init(), which subclasses call from their own
        constructor body. The declaration is lazy now, so the probe happens on first deferral.
        """
        class LateInitNode(PackmlNode):
            def __init__(self):
                super().__init__('late_init_node')
                # Assigned AFTER super().__init__() on purpose -- that ordering is the test.
                self._deferred_states = {State.RESETTING}

            def defers_completion(self, state):
                return state in self._deferred_states

        node = LateInitNode()
        try:
            assert node.defers_completion(State.RESETTING) is True
            assert node.defers_completion(State.STARTING) is False
        finally:
            node.destroy_node()

    def test_report_after_the_goal_is_gone_is_harmless(self, client_node, executor):
        """A handle whose goal has already timed out is inert: reporting through it must not
        raise, and there is nothing left for it to resolve."""
        class DeferringNode(PackmlNode):
            def __init__(self):
                super().__init__(
                    'gone_goal_node',
                    parameter_overrides=[Parameter(PARAM_DEFERRED_COMPLETION_TIMEOUT_MS, value=300)])
                self.completion = None

            def defers_completion(self, state):
                return state == State.RESETTING

            def on_deferred_work(self, state, completion):
                self.completion = completion

        node = DeferringNode()
        executor.add_node(node)

        action_client = ActionClient(
            client_node, StateTransitionAction, f'/gone_goal_node/{STATE_TRANSITION_ACTION}')
        assert action_client.wait_for_server(timeout_sec=2.0)

        goal = StateTransitionAction.Goal()
        goal.state.val = int(State.RESETTING)
        send_future = action_client.send_goal_async(goal)
        deadline = time.monotonic() + 2.0
        while not send_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        goal_handle = send_future.result()
        assert goal_handle is not None and goal_handle.accepted

        # Ride out the node's own 300ms deferred-completion timeout, which aborts the goal.
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + 2.0
        while not result_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert result_future.done(), 'the goal never timed out'
        assert result_future.result().result.success is False

        assert node.completion is not None, 'the node was never asked to do deferred work'
        assert node.completion.abandoned is True
        node.completion.report(True)  # inert: nothing is waiting on it

        action_client.destroy()
        node.destroy_node()

    def test_stale_handle_does_not_resolve_a_later_goal(self, client_node, executor):
        """An earlier goal's handle must never resolve a later goal for the SAME state.

        The Python mirror of the C++ MonkeyResetAbortReset_StaleReportDoesNotResolveLaterReset:
        work that outlives the goal that asked for it reports whenever it finishes, and a state
        name recurs across an operator's repeated commands, so a name-keyed completion slot let
        the first goal's report satisfy the second goal's wait. Deterministic here -- the first
        goal is driven to its timeout before the second one is even sent.
        """
        class DeferringNode(PackmlNode):
            def __init__(self):
                super().__init__(
                    'stale_handle_node',
                    parameter_overrides=[Parameter(PARAM_DEFERRED_COMPLETION_TIMEOUT_MS, value=300)])
                self.completions = []

            def defers_completion(self, state):
                return state == State.RESETTING

            def on_deferred_work(self, state, completion):
                self.completions.append(completion)

        node = DeferringNode()
        executor.add_node(node)

        action_client = ActionClient(
            client_node, StateTransitionAction, f'/stale_handle_node/{STATE_TRANSITION_ACTION}')
        assert action_client.wait_for_server(timeout_sec=2.0)

        def send_resetting_goal():
            goal = StateTransitionAction.Goal()
            goal.state.val = int(State.RESETTING)
            send_future = action_client.send_goal_async(goal)
            deadline = time.monotonic() + 2.0
            while not send_future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
            handle = send_future.result()
            assert handle is not None and handle.accepted
            return handle

        # Goal A: accepted, then left to ride out the node's own 300ms timeout.
        goal_a = send_resetting_goal()
        result_a = goal_a.get_result_async()
        deadline = time.monotonic() + 2.0
        while not result_a.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert result_a.done(), 'goal A never timed out'
        assert len(node.completions) == 1

        # Goal B: a fresh RESETTING, same state name as goal A.
        goal_b = send_resetting_goal()
        result_b = goal_b.get_result_async()
        deadline = time.monotonic() + 2.0
        while len(node.completions) < 2 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert len(node.completions) == 2, 'goal B was never asked to do deferred work'

        # Goal A's work finishes late and reports. It must not resolve goal B.
        node.completions[0].report(True)
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert not result_b.done(), "goal A's stale report resolved goal B"

        # Goal B's own report still resolves it normally.
        node.completions[1].report(True)
        deadline = time.monotonic() + 2.0
        while not result_b.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert result_b.done()
        assert result_b.result().result.success is True

        action_client.destroy()
        node.destroy_node()

    def test_second_report_for_the_same_goal_is_discarded(self, client_node, executor):
        """First report wins: a contradictory second report for the same goal is discarded."""
        class DeferringNode(PackmlNode):
            def __init__(self):
                super().__init__(
                    'double_report_node',
                    parameter_overrides=[Parameter(PARAM_DEFERRED_COMPLETION_TIMEOUT_MS, value=3000)])
                self.completion = None

            def defers_completion(self, state):
                return state == State.RESETTING

            def on_deferred_work(self, state, completion):
                self.completion = completion

        node = DeferringNode()
        executor.add_node(node)

        action_client = ActionClient(
            client_node, StateTransitionAction, f'/double_report_node/{STATE_TRANSITION_ACTION}')
        assert action_client.wait_for_server(timeout_sec=2.0)

        goal = StateTransitionAction.Goal()
        goal.state.val = int(State.RESETTING)
        send_future = action_client.send_goal_async(goal)
        deadline = time.monotonic() + 2.0
        while not send_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        goal_handle = send_future.result()
        assert goal_handle is not None and goal_handle.accepted

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + 2.0
        while node.completion is None and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert node.completion is not None, 'the node was never asked to do deferred work'

        node.completion.report(True)
        node.completion.report(success=False, error_code=99, message='stale second report')

        deadline = time.monotonic() + 2.0
        while not result_future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        assert result_future.done()
        result = result_future.result().result
        assert result.success is True, 'the discarded second report changed the outcome'
        assert result.error_code != 99

        action_client.destroy()
        node.destroy_node()
