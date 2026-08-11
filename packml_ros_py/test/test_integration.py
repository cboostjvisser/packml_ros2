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

"""Integration test: Python PackmlNode managed by C++ packml_ros_node manager.

This test launches the C++ manager and a Python PackML node, then verifies
the manager can command state transitions on the Python node via real
ROS2 service calls.  State is verified by subscribing to the packml_status
topic published by the manager.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from packml_msgs.srv import StateChange
from packml_msgs.msg import Status


@pytest.mark.launch_test
def generate_test_description():
    """Launch the C++ manager and a Python PackML node."""
    py_node = launch_ros.actions.Node(
        package='packml_ros_py',
        executable='example_packml_node',
        name='py_equipment_module',
        output='screen',
    )

    manager_node = launch_ros.actions.Node(
        package='packml_ros',
        executable='packml_ros_node',
        name='packml_manager',
        output='screen',
        parameters=[{
            'node_names': ['py_equipment_module'],
        }],
    )

    return launch.LaunchDescription([
        py_node,
        manager_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'manager': manager_node, 'py_node': py_node}


class TestPythonNodeManagedByCppManager(unittest.TestCase):
    """Verify the C++ manager can control the Python node end-to-end."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('integration_test_node')
        cls.state_change_client = cls.node.create_client(
            StateChange,
            '/packml_manager/changeState',
        )
        cls.last_status = None
        # Match the manager's LATCHED status publisher (TRANSIENT_LOCAL + RELIABLE, depth 1 --
        # see where status_pub_ is created in packml_interface.hpp). This subscription used
        # qos_profile_sensor_data, which is VOLATILE: compatible enough to match, but a volatile
        # subscriber is never given the retained sample. Status is published only on CHANGE, and
        # the manager publishes STOPPED exactly once at boot, so whether test_01 ever saw it came
        # down to whether this subscription finished matching before that single publish. Alone it
        # usually won; behind another test module's import time it usually lost, and then no later
        # message could rescue it because the machine does not re-enter STOPPED on its own.
        cls.status_sub = cls.node.create_subscription(
            Status,
            'packml_status',
            cls._on_status,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_status(cls, msg):
        cls.last_status = msg

    def _send_command(self, command, timeout=10.0):
        """Send a StateChange command and return the response."""
        req = StateChange.Request()
        req.command = command
        future = self.state_change_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        self.assertTrue(future.done(), 'Service call timed out')
        return future.result()

    def _spin_until_state(self, expected_state_val, timeout=5.0):
        """Spin until last_status reports the expected state, or timeout."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.last_status and self.last_status.state.val == expected_state_val:
                return True
        return False

    def test_01_manager_service_available(self):
        """Manager's changeState service should come up, and the system should settle
        into its initial STOPPED state before any test issues its own transition.

        The state-transition action's goal/cancel/result/feedback/status entities take
        longer to discover across the manager<->EM pair than the single service they
        replaced -- waiting only for the manager's own changeState service (as before)
        let test_02 fire RESET while that discovery, and the activation-time STOPPED
        fanout it gates, were still in flight. A RESETTING goal arriving at the EM
        before it had processed the STOPPED status update hit TransitionGuard's
        "already in progress" rejection, since that flag only clears on an observed
        status change -- not on the EM's own action goal completing.
        """
        self.assertTrue(
            self.state_change_client.wait_for_service(timeout_sec=10.0),
            'Manager changeState service did not become available',
        )
        got_stopped = self._spin_until_state(2, timeout=10.0)  # STOPPED
        self.assertTrue(got_stopped, 'System did not settle into STOPPED at startup')
        # This test's own status subscription observing STOPPED confirms the manager
        # has published it, but not that the EM (a separate process) has finished
        # processing that same message yet -- give it a brief margin too.
        time.sleep(0.3)

    def test_02_reset_to_idle(self):
        """RESET should bring the system through RESETTING → IDLE."""
        result = self._send_command(StateChange.Request.RESET)
        self.assertIsNotNone(result)
        # Wait for IDLE (state val 4)
        got_idle = self._spin_until_state(4, timeout=10.0)
        self.assertTrue(got_idle, 'System did not reach IDLE after RESET')

    def test_03_start_to_execute(self):
        """START from IDLE should reach EXECUTE."""
        result = self._send_command(StateChange.Request.START)
        self.assertIsNotNone(result)
        # Wait for EXECUTE (state val 6)
        got_execute = self._spin_until_state(6, timeout=10.0)
        self.assertTrue(got_execute, 'System did not reach EXECUTE after START')

    def test_04_hold_and_unhold(self):
        """HOLD should reach HELD, UNHOLD should return to EXECUTE."""
        result = self._send_command(StateChange.Request.HOLD)
        self.assertIsNotNone(result)
        # Wait for HELD (state val 11)
        got_held = self._spin_until_state(11, timeout=10.0)
        self.assertTrue(got_held, 'System did not reach HELD after HOLD')

        result = self._send_command(StateChange.Request.UNHOLD)
        self.assertIsNotNone(result)
        # Wait for EXECUTE (state val 6)
        got_execute = self._spin_until_state(6, timeout=10.0)
        self.assertTrue(got_execute, 'System did not reach EXECUTE after UNHOLD')

    def test_05_stop_to_stopped(self):
        """STOP should bring the system to STOPPED."""
        result = self._send_command(StateChange.Request.STOP)
        self.assertIsNotNone(result)
        # Wait for STOPPED (state val 2)
        got_stopped = self._spin_until_state(2, timeout=10.0)
        self.assertTrue(got_stopped, 'System did not reach STOPPED after STOP')

    def test_06_abort_and_clear(self):
        """ABORT should reach ABORTED, CLEAR should reach STOPPED."""
        result = self._send_command(StateChange.Request.ABORT)
        self.assertIsNotNone(result)
        # Wait for ABORTED (state val 9)
        got_aborted = self._spin_until_state(9, timeout=10.0)
        self.assertTrue(got_aborted, 'System did not reach ABORTED after ABORT')

        result = self._send_command(StateChange.Request.CLEAR)
        self.assertIsNotNone(result)
        # Wait for STOPPED (state val 2)
        got_stopped = self._spin_until_state(2, timeout=10.0)
        self.assertTrue(got_stopped, 'System did not reach STOPPED after CLEAR')
