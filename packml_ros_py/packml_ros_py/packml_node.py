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

"""PackmlNode base class for Python-based PackML managed nodes.

This mirrors the C++ PackmlNodeInterface: it exposes the services that the
PackML manager calls to coordinate state/mode transitions, and subscribes
to the manager's status topic to track current system state.

The shared protocol logic (transition coordination + heartbeat state) is
delegated to the C++ PackmlNodeProtocol class via pybind11, ensuring
single-source-of-truth behaviour between C++ and Python nodes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from packml_msgs.srv import StateTransition, ModeTransition
from packml_msgs.msg import Status, NodeHealth, NodeHeartbeat

from packml_ros_py.enums import State
from packml_ros_py._packml_bindings import (
    PackmlNodeProtocol as _Protocol,
    STATE_TRANSITION_SERVICE,
    MODE_TRANSITION_SERVICE,
    STATUS_TOPIC,
    HEARTBEAT_TOPIC,
    PARAM_HEARTBEAT_INTERVAL_MS,
)


class PackmlNode(Node):
    """Base class for Python PackML managed nodes.

    Subclass this and override the callback methods to implement your
    equipment module logic. The manager will call the state/mode transition
    services on this node.

    Usage::

        class MyModule(PackmlNode):
            def __init__(self):
                super().__init__('my_module')

            def on_state_transition_request(self, target_state: State) -> bool:
                # Return True to accept the transition
                return True

            def on_mode_transition_request(self, target_mode: int) -> bool:
                return True

            def on_packml_status_changed(self, status: Status) -> None:
                pass
    """

    def __init__(self, node_name: str, **kwargs):
        super().__init__(node_name, **kwargs)

        # C++ PackmlNodeProtocol — single source of truth for shared protocol logic
        self._protocol = _Protocol()

        # Service: ~/packml_state_transition
        self._state_transition_srv = self.create_service(
            StateTransition,
            '~/' + STATE_TRANSITION_SERVICE,
            self._handle_state_transition,
        )

        # Service: ~/packml_mode_transition
        self._mode_transition_srv = self.create_service(
            ModeTransition,
            '~/' + MODE_TRANSITION_SERVICE,
            self._handle_mode_transition,
        )

        # Subscription: packml_status (from manager). The manager publishes status
        # latched (TRANSIENT_LOCAL + RELIABLE) so a node that (re)starts after the
        # manager has already published immediately receives the current state.
        # Match that here (parity with the C++ PackmlNodeInterface).
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # Sensor-style QoS (best-effort) reused for the heartbeat publisher below.
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._status_sub = self.create_subscription(
            Status,
            STATUS_TOPIC,
            self._handle_status,
            status_qos,
        )

        # --- Heartbeat publisher (mirrors C++ PackmlNodeInterface) ---
        heartbeat_interval_ms = self.declare_parameter(
            PARAM_HEARTBEAT_INTERVAL_MS, 1000).value
        heartbeat_interval_ms = heartbeat_interval_ms if heartbeat_interval_ms > 0 else 1000

        self._protocol.heartbeat.init(self.get_name(), heartbeat_interval_ms)

        # Sensor-style QoS (best-effort, keep-last) — must match the manager's
        # heartbeat subscription, or QoS-incompatibility silently drops delivery.
        self._heartbeat_pub = self.create_publisher(NodeHeartbeat, '~/' + HEARTBEAT_TOPIC, sensor_qos)
        self._heartbeat_timer = self.create_timer(
            heartbeat_interval_ms / 1000.0,
            self._publish_heartbeat,
        )

        self.get_logger().info('PackmlNode initialized')

    # ─── Properties ──────────────────────────────────────────────────────

    @property
    def current_state(self) -> State:
        """Current PackML state as reported by the manager."""
        return State(int(self._protocol.transitions.current_state))

    @property
    def current_mode(self) -> int:
        """Current PackML mode as reported by the manager."""
        return self._protocol.transitions.current_mode

    # ─── Callbacks for subclasses to override ────────────────────────────

    def on_state_transition_request(self, target_state: State) -> bool:
        """Called when the manager requests a state transition.

        Override in subclass. Return True to accept, False to reject.
        """
        return True

    def on_mode_transition_request(self, target_mode: int) -> bool:
        """Called when the manager requests a mode transition.

        Override in subclass. Return True to accept, False to reject.
        """
        return True

    def on_packml_status_changed(self, status: Status) -> None:
        """Called when the manager publishes a new status.

        Override in subclass to react to system-wide state/mode updates.
        """
        pass

    # ─── Health / heartbeat API ───────────────────────────────────────────

    def get_health_status(self) -> NodeHealth:
        """Return the current health of this Equipment Module.

        Override in subclass to report real conditions.
        The base implementation returns HEALTHY / NONE.
        """
        msg = NodeHealth()
        msg.status = NodeHealth.HEALTHY
        msg.action = NodeHealth.NONE
        return msg

    def _set_heartbeat_active(self, active: bool) -> None:
        """Pause or resume heartbeat publishing.

        Intended for derived test/demo Equipment Modules to simulate a crashed or
        silent node (no heartbeat = timeout in the HealthMonitor) — not part of the
        public API (mirrors the protected `set_heartbeat_active` in the C++
        PackmlNodeInterface).
        """
        self._protocol.heartbeat.set_active(active)

    def _make_heartbeat(self, health: NodeHealth) -> NodeHeartbeat:
        """Assemble a NodeHeartbeat with the standard header (node_name, next sequence,
        interval) and the given health.  Single source for both post_event() and the
        periodic timer; calls next_sequence() exactly once per published heartbeat
        (mirrors the C++ PackmlNodeInterface::make_heartbeat)."""
        hb = NodeHeartbeat()
        hb.node_name = self._protocol.heartbeat.node_name
        hb.sequence_number = self._protocol.heartbeat.next_sequence()
        hb.heartbeat_interval_ms = self._protocol.heartbeat.interval_ms
        hb.health = health
        return hb

    def post_event(self, health: NodeHealth) -> None:
        """Immediately publish a heartbeat with the given health state.

        Bypasses the periodic timer.  Use for safety-critical events (e.g. E-stop)
        where waiting up to heartbeat_interval_ms for the next tick is unacceptable.
        The event is *latched*: the periodic publisher repeats this health on every
        subsequent tick (instead of calling get_health_status()), so a transient
        getter cannot flap the alarm/state.  Posting a healthy/NONE event clears it.
        """
        if health.action == NodeHealth.NONE:
            self._protocol.heartbeat.clear_latch()
        else:
            self._protocol.heartbeat.set_latch(
                health.status, health.action, health.error_code, health.message,
                health.instance_id)
        self._heartbeat_pub.publish(self._make_heartbeat(health))

    def _publish_heartbeat(self) -> None:
        """Periodic heartbeat callback — called by the internal timer."""
        if not self._protocol.heartbeat.is_active:
            return
        # One atomic snapshot of the latch (no torn read vs a concurrent post_event),
        # matching the C++ periodic publisher.
        latch = self._protocol.heartbeat.latch_snapshot()
        if latch.active:
            # A post_event() fault is latched — repeat it, don't poll the getter.
            health = NodeHealth()
            health.status = latch.status
            health.action = latch.action
            health.error_code = latch.error_code
            health.message = latch.message
            health.instance_id = latch.instance_id
        else:
            health = self.get_health_status()
        self._heartbeat_pub.publish(self._make_heartbeat(health))

    # ─── Internal service handlers ───────────────────────────────────────

    def _handle_state_transition(self, request, response):
        """Handle ~/packml_state_transition service call from manager."""
        target = State(request.state.val)
        result = self._protocol.transitions.request_state(target)

        if result.already_there:
            self.get_logger().info(f'Already in state: {target.name}')
            response.success = True
            return response

        if result.error:
            self.get_logger().warn(result.error)

        # Ask subclass whether to accept
        if result.accepted and self.on_state_transition_request(target):
            self.get_logger().info(f'Approved state transition to {target.name}')
            response.success = True
        else:
            error_msg = 'Node rejected state transition'
            response.success = False
            response.message = error_msg
            self.get_logger().warn(error_msg)

        return response

    def _handle_mode_transition(self, request, response):
        """Handle ~/packml_mode_transition service call from manager."""
        target_mode = int(request.mode.val)
        result = self._protocol.transitions.request_mode(target_mode)

        if result.already_there:
            self.get_logger().info(f'Already in mode: {target_mode}')
            response.success = True
            return response

        if result.error:
            self.get_logger().warn(result.error)

        # Ask subclass whether to accept
        if result.accepted and self.on_mode_transition_request(target_mode):
            self.get_logger().info(f'Approved mode transition to {target_mode}')
            response.success = True
        else:
            error_msg = 'Node rejected mode transition'
            response.success = False
            response.message = error_msg
            self.get_logger().warn(error_msg)

        return response

    def _handle_status(self, msg: Status):
        """Handle packml_status subscription message from manager."""
        changed = self._protocol.transitions.on_status_update(State(msg.state.val), int(msg.mode.val))

        if changed:
            self.get_logger().debug(
                f'Status updated - state: {State(msg.state.val).name}, mode: {msg.mode.val}'
            )
            self.on_packml_status_changed(msg)
