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

The transition validation logic is delegated to the C++ TransitionGuard
class via pybind11, ensuring single-source-of-truth protocol behavior
between C++ and Python nodes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from packml_msgs.srv import StateTransition, ModeTransition
from packml_msgs.msg import Status

from packml_ros_py.enums import State
from packml_ros_py._packml_bindings import (
    TransitionGuard as _TransitionGuard,
    STATE_TRANSITION_SERVICE,
    MODE_TRANSITION_SERVICE,
    STATUS_TOPIC,
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

        # C++ TransitionGuard — single source of truth for protocol logic
        self._guard = _TransitionGuard()

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

        # Subscription: packml_status (from manager)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._status_sub = self.create_subscription(
            Status,
            STATUS_TOPIC,
            self._handle_status,
            sensor_qos,
        )

        self.get_logger().info('PackmlNode initialized')

    # ─── Properties ──────────────────────────────────────────────────────

    @property
    def current_state(self) -> State:
        """Current PackML state as reported by the manager."""
        return State(int(self._guard.current_state))

    @property
    def current_mode(self) -> int:
        """Current PackML mode as reported by the manager."""
        return self._guard.current_mode

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

    # ─── Internal service handlers ───────────────────────────────────────

    def _handle_state_transition(self, request, response):
        """Handle ~/packml_state_transition service call from manager."""
        target = State(request.state.val)
        result = self._guard.request_state(target)

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
        result = self._guard.request_mode(target_mode)

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
        changed = self._guard.on_status_update(State(msg.state.val), int(msg.mode.val))

        if changed:
            self.get_logger().debug(
                f'Status updated - state: {State(msg.state.val).name}, mode: {msg.mode.val}'
            )
            self.on_packml_status_changed(msg)
