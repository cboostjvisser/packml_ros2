#!/usr/bin/env python3
#
# Software License Agreement
# Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Contributors: Dejanira Araiza Illan
#
# ---
# PackML Sender Tests
#
# Tests the packml_plc_sender module against a mock PackML OPC UA server.
#
# Test categories:
# 1. Unit tests: DriverSender instantiation, connect, and exit behavior
# 2. Valid transition tests: command sequence drives expected mock PLC states
# 3. Invalid transition tests: illegal commands do not change current state
# ---
import unittest

from packml_msgs.srv import StateChange

import threading
import time
from packml_plc.packml_plc_sender import DriverSender
from packml_plc.mock_plc import PackMLMockPLC

# OPC UA default port for all tests
MOCK_PLC_ENDPOINT = 'opc.tcp://127.0.0.1:4840'
NODE_ID_PREFIX = 'ns=3;s="PackML_Status"."EM00"."Unit"'


class TestMethods(unittest.TestCase):

    def test_driversender_illegal_transition(self):
        """
        Try to START from ABORTED state, which is illegal per PackML.
        The OPC UA write may succeed, but the state should NOT change.
        """
        import rclpy
        rclpy.init(args=None)
        plc = PackMLMockPLC(endpoint=MOCK_PLC_ENDPOINT)
        server_thread = threading.Thread(target=plc.start, daemon=True)
        server_thread.start()
        time.sleep(1.0)
        try:
            driver = DriverSender(endpoint=MOCK_PLC_ENDPOINT)
            driver.client.connect()
            res = StateChange.Response()
            req = StateChange.Request()
            # Set PLC to ABORTED state (using set_state helper)
            plc.set_state('Aborted')
            time.sleep(0.1)
            # Try to START from ABORTED (should be ignored per PackML)
            req.command = StateChange.Request.START
            driver.trans_request(req, res)
            time.sleep(0.2)  # Give mock PLC time to process
            # Verify state did NOT change to Execute (PackML enforcement)
            node_execute = driver.client.get_node(f'{NODE_ID_PREFIX}."Execute"')
            self.assertFalse(node_execute.get_value(), "State should NOT be Execute after illegal START from ABORTED")
            # Verify we're still in ABORTED
            node_aborted = driver.client.get_node(f'{NODE_ID_PREFIX}."Aborted"')
            self.assertTrue(node_aborted.get_value(), "State should still be ABORTED after illegal START command")
            driver.client.disconnect()
        finally:
            plc.stop()
            time.sleep(0.5)
            rclpy.shutdown()

    def test_driversender(self):
        import rclpy
        rclpy.init(args=None)
        driver = DriverSender(endpoint=MOCK_PLC_ENDPOINT)
        self.assertNotEqual(driver.client, [])
        rclpy.shutdown()

    def test_driversender_connect(self):
        import rclpy
        rclpy.init(args=None)
        plc = PackMLMockPLC(endpoint=MOCK_PLC_ENDPOINT)
        server_thread = threading.Thread(target=plc.start, daemon=True)
        server_thread.start()
        time.sleep(0.5)
        try:
            driver = DriverSender(endpoint=MOCK_PLC_ENDPOINT)
            driver.connect()
            node_stopped = driver.client.get_node(f'{NODE_ID_PREFIX}."Stopped"')
            self.assertTrue(node_stopped.get_value(), "Expected initial STOPPED state to be True")
            driver.client.disconnect()
        finally:
            plc.stop()
            time.sleep(0.3)
            rclpy.shutdown()

    def test_driversender_exit(self):
        import rclpy
        rclpy.init(args=None)
        plc = PackMLMockPLC(endpoint=MOCK_PLC_ENDPOINT)
        server_thread = threading.Thread(target=plc.start, daemon=True)
        server_thread.start()
        time.sleep(0.5)
        try:
            driver = DriverSender(endpoint=MOCK_PLC_ENDPOINT)
            driver.connect()
            # Connect first so __exit__ has a live connection to close.
            driver.__exit__(1, 1, 1)
        finally:
            plc.stop()
            time.sleep(0.3)
            rclpy.shutdown()

    def test_driversender_trans_request(self):
        import rclpy
        rclpy.init(args=None)
        # Start the mock PLC server in a background thread
        plc = PackMLMockPLC(endpoint=MOCK_PLC_ENDPOINT)
        server_thread = threading.Thread(target=plc.start, daemon=True)
        server_thread.start()
        time.sleep(1.0)  # Give server time to start
        try:
            driver = DriverSender(endpoint=MOCK_PLC_ENDPOINT)
            driver.client.connect()
            res = StateChange.Response()
            req = StateChange.Request()
            # Test a VALID PackML transition sequence
            # Mock PLC starts in STOPPED state
            valid_sequence = [
                # (command, expected_state_after)
                (StateChange.Request.RESET, 'Idle'),        # STOPPED -> IDLE
                (StateChange.Request.START, 'Execute'),     # IDLE -> EXECUTE
                (StateChange.Request.HOLD, 'Held'),         # EXECUTE -> HELD
                (StateChange.Request.UNHOLD, 'Execute'),    # HELD -> EXECUTE
                (StateChange.Request.SUSPEND, 'Suspended'), # EXECUTE -> SUSPENDED
                (StateChange.Request.UNSUSPEND, 'Execute'), # SUSPENDED -> EXECUTE
                (StateChange.Request.STOP, 'Stopped'),      # EXECUTE -> STOPPED
                (StateChange.Request.RESET, 'Idle'),        # STOPPED -> IDLE (repeat)
                (StateChange.Request.START, 'Execute'),     # IDLE -> EXECUTE (repeat)
                (StateChange.Request.ABORT, 'Aborted'),     # Any -> ABORTED
                (StateChange.Request.CLEAR, 'Stopped'),     # ABORTED -> STOPPED
            ]
            all_states = ['Idle', 'Execute', 'Stopped', 'Held', 'Suspended', 'Aborted']
            for cmd, expected_state in valid_sequence:
                req.command = cmd
                driver.trans_request(req, res)
                self.assertTrue(res.success, f"Command {cmd} should succeed")
                # Give the mock PLC a moment to process
                time.sleep(0.1)
                # Check that only the expected state is True
                for state_name in all_states:
                    node = driver.client.get_node(f'{NODE_ID_PREFIX}."{state_name}"')
                    value = node.get_value()
                    if state_name == expected_state:
                        self.assertTrue(value, f"After {cmd}, {expected_state} should be True")
                    else:
                        self.assertFalse(value, f"After {cmd}, only {expected_state} should be True, but {state_name} is also True")
            # Test NO_COMMAND and invalid command (should fail)
            req.command = StateChange.Request.NO_COMMAND
            driver.trans_request(req, res)
            self.assertFalse(res.success)
            req.command = 100  # INVALID/UNRECOGNIZED
            driver.trans_request(req, res)
            self.assertFalse(res.success)
            driver.client.disconnect()
        finally:
            plc.stop()
            time.sleep(0.5)
            rclpy.shutdown()


class TestInvalidTransitions(unittest.TestCase):
    """Test all invalid PackML transitions."""

    @classmethod
    def setUpClass(cls):
        import rclpy
        rclpy.init(args=None)
        cls.plc = PackMLMockPLC(endpoint=MOCK_PLC_ENDPOINT)
        server_thread = threading.Thread(target=cls.plc.start, daemon=True)
        server_thread.start()
        time.sleep(1.0)

    @classmethod
    def tearDownClass(cls):
        import rclpy
        cls.plc.stop()
        time.sleep(0.5)
        rclpy.shutdown()

    def setUp(self):
        self.driver = DriverSender(endpoint=MOCK_PLC_ENDPOINT)
        self.driver.client.connect()

    def tearDown(self):
        self.driver.client.disconnect()

    def _send_command(self, cmd):
        req = StateChange.Request()
        res = StateChange.Response()
        req.command = cmd
        self.driver.trans_request(req, res)
        time.sleep(0.15)
        return res

    def test_stopped_invalid_commands(self):
        """From STOPPED: START, HOLD, UNHOLD, SUSPEND, UNSUSPEND, CLEAR are invalid."""
        self.plc.set_state('Stopped')
        time.sleep(0.1)

        invalid_cmds = [
            StateChange.Request.START,      # Must RESET first
            StateChange.Request.HOLD,       # Only from EXECUTE
            StateChange.Request.UNHOLD,     # Only from HELD
            StateChange.Request.SUSPEND,    # Only from EXECUTE
            StateChange.Request.UNSUSPEND,  # Only from SUSPENDED
            StateChange.Request.CLEAR,      # Only from ABORTED
        ]
        for cmd in invalid_cmds:
            self._send_command(cmd)
            self.assertEqual(self.plc.get_state(), 'Stopped',
                f"State should remain STOPPED after invalid cmd {cmd}")

    def test_idle_invalid_commands(self):
        """From IDLE: HOLD, UNHOLD, SUSPEND, UNSUSPEND, CLEAR, RESET are invalid."""
        self.plc.set_state('Idle')
        time.sleep(0.1)

        invalid_cmds = [
            StateChange.Request.HOLD,       # Only from EXECUTE
            StateChange.Request.UNHOLD,     # Only from HELD
            StateChange.Request.SUSPEND,    # Only from EXECUTE
            StateChange.Request.UNSUSPEND,  # Only from SUSPENDED
            StateChange.Request.CLEAR,      # Only from ABORTED
            StateChange.Request.RESET,      # Only from STOPPED/COMPLETE
        ]
        for cmd in invalid_cmds:
            self._send_command(cmd)
            self.assertEqual(self.plc.get_state(), 'Idle',
                f"State should remain IDLE after invalid cmd {cmd}")

    def test_execute_invalid_commands(self):
        """From EXECUTE: RESET, START, UNHOLD, UNSUSPEND, CLEAR are invalid."""
        self.plc.set_state('Execute')
        time.sleep(0.1)

        invalid_cmds = [
            StateChange.Request.RESET,      # Only from STOPPED/COMPLETE
            StateChange.Request.START,      # Only from IDLE
            StateChange.Request.UNHOLD,     # Only from HELD
            StateChange.Request.UNSUSPEND,  # Only from SUSPENDED
            StateChange.Request.CLEAR,      # Only from ABORTED
        ]
        for cmd in invalid_cmds:
            self._send_command(cmd)
            self.assertEqual(self.plc.get_state(), 'Execute',
                f"State should remain EXECUTE after invalid cmd {cmd}")

    def test_aborted_only_clear_allowed(self):
        """From ABORTED: Only CLEAR is valid."""
        self.plc.set_state('Aborted')
        time.sleep(0.1)

        invalid_cmds = [
            StateChange.Request.RESET,
            StateChange.Request.START,
            StateChange.Request.STOP,
            StateChange.Request.HOLD,
            StateChange.Request.UNHOLD,
            StateChange.Request.SUSPEND,
            StateChange.Request.UNSUSPEND,
            StateChange.Request.ABORT,  # Already aborted
        ]
        for cmd in invalid_cmds:
            self._send_command(cmd)
            self.assertEqual(self.plc.get_state(), 'Aborted',
                f"State should remain ABORTED after invalid cmd {cmd}")

        # CLEAR should work
        self._send_command(StateChange.Request.CLEAR)
        self.assertEqual(self.plc.get_state(), 'Stopped',
            "CLEAR should transition from ABORTED to STOPPED")

    def test_held_only_unhold_stop_abort_allowed(self):
        """From HELD: Only UNHOLD, STOP, ABORT are valid."""
        self.plc.set_state('Held')
        time.sleep(0.1)

        invalid_cmds = [
            StateChange.Request.RESET,
            StateChange.Request.START,
            StateChange.Request.HOLD,       # Already HELD
            StateChange.Request.SUSPEND,
            StateChange.Request.UNSUSPEND,
            StateChange.Request.CLEAR,
        ]
        for cmd in invalid_cmds:
            self._send_command(cmd)
            self.assertEqual(self.plc.get_state(), 'Held',
                f"State should remain HELD after invalid cmd {cmd}")

    def test_suspended_only_unsuspend_stop_abort_allowed(self):
        """From SUSPENDED: Only UNSUSPEND, STOP, ABORT are valid."""
        self.plc.set_state('Suspended')
        time.sleep(0.1)

        invalid_cmds = [
            StateChange.Request.RESET,
            StateChange.Request.START,
            StateChange.Request.HOLD,
            StateChange.Request.UNHOLD,
            StateChange.Request.SUSPEND,    # Already SUSPENDED
            StateChange.Request.CLEAR,
        ]
        for cmd in invalid_cmds:
            self._send_command(cmd)
            self.assertEqual(self.plc.get_state(), 'Suspended',
                f"State should remain SUSPENDED after invalid cmd {cmd}")
