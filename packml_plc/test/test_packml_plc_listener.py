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
# PackML Listener Tests
#
# Tests the packml_plc_listener module against a mock PackML OPC UA server.
#
# Test categories:
# 1. Unit tests: HelloClient, DriverListener instantiation
# 2. Integration tests: plc_listener reading states from mock PLC
# 3. State transition tests: Mock PLC state changes reflected in listener
# ---

import threading
import time
import unittest
from unittest.mock import MagicMock

from packml_msgs.srv import AllStatus
from packml_plc.packml_plc_listener import DriverListener
from packml_plc.packml_plc_listener import HelloClient
from packml_plc.packml_plc_listener import plc_listener
from packml_plc.packml_plc_listener import PACKML_STATE_NAMES
import packml_plc.packml_plc_listener as listener_module
import rclpy

# Import mock PLC from package
from packml_plc.mock_plc import PackMLMockPLC, PACKML_STATES

# OPC UA default port for all tests
MOCK_PLC_ENDPOINT = 'opc.tcp://127.0.0.1:4840'


class TestUnitMethods(unittest.TestCase):
    """Unit tests - no mock PLC required."""

    def test_helloclient_instantiation(self):
        """Test HelloClient can be instantiated with custom endpoint."""
        client = HelloClient(endpoint=MOCK_PLC_ENDPOINT)
        self.assertIsNotNone(client)

    def test_driverlistener_instantiation(self):
        """Test DriverListener node creates service."""
        rclpy.init(args=None)
        try:
            driver = DriverListener()
            self.assertIsNotNone(driver.srv)
        finally:
            rclpy.shutdown()

    def test_packml_state_names_count(self):
        """
        Verify PACKML_STATE_NAMES has exactly 17 states.
        """
        self.assertEqual(len(PACKML_STATE_NAMES), 17)
        # Verify key states are present
        self.assertIn('Stopped', PACKML_STATE_NAMES)
        self.assertIn('Idle', PACKML_STATE_NAMES)
        self.assertIn('Execute', PACKML_STATE_NAMES)
        self.assertIn('Aborted', PACKML_STATE_NAMES)


class TestIntegrationWithMockPLC(unittest.TestCase):
    """Integration tests using the mock PLC server."""

    @classmethod
    def setUpClass(cls):
        """Start the mock PLC server once for all tests in this class."""
        cls.plc = PackMLMockPLC(endpoint=MOCK_PLC_ENDPOINT)
        cls.plc.start()
        time.sleep(0.5)  # Allow server to start

    @classmethod
    def tearDownClass(cls):
        """Stop the mock PLC server."""
        cls.plc.stop()
        time.sleep(0.3)

    def test_listener_reads_initial_stopped_state(self):
        """
        Test that plc_listener correctly reads STOPPED as initial state.
        """
        # Reset to initial state
        self.plc.set_state('Stopped')

        # Run listener briefly
        stop_event = threading.Event()
        listener_thread = threading.Thread(
            target=plc_listener,
            args=(stop_event, MOCK_PLC_ENDPOINT)
        )
        listener_thread.start()
        time.sleep(0.3)  # Allow one poll cycle
        stop_event.set()
        listener_thread.join(timeout=2)

        # Verify newvals reflects STOPPED (index 0)
        self.assertTrue(listener_module.newvals[0],
            "Stopped should be True (index 0)")
        # Verify only ONE state is active (PackML rule)
        active_count = sum(1 for v in listener_module.newvals if v)
        self.assertEqual(active_count, 1,
            f"Only ONE state should be active, got {active_count}")

    def test_listener_reads_idle_state(self):
        """
        Test that plc_listener correctly reads IDLE state.
        """
        # Set mock to IDLE
        self.plc.set_state('Idle')

        # Run listener briefly
        stop_event = threading.Event()
        listener_thread = threading.Thread(
            target=plc_listener,
            args=(stop_event, MOCK_PLC_ENDPOINT)
        )
        listener_thread.start()
        time.sleep(0.3)
        stop_event.set()
        listener_thread.join(timeout=2)

        # Verify newvals reflects IDLE (index 1)
        self.assertTrue(listener_module.newvals[1],
            "Idle should be True (index 1)")
        self.assertFalse(listener_module.newvals[0],
            "Stopped should be False")

    def test_listener_reads_execute_state(self):
        """
        Test that plc_listener correctly reads EXECUTE state.
        """
        # Set mock to EXECUTE
        self.plc.set_state('Execute')

        stop_event = threading.Event()
        listener_thread = threading.Thread(
            target=plc_listener,
            args=(stop_event, MOCK_PLC_ENDPOINT)
        )
        listener_thread.start()
        time.sleep(0.3)
        stop_event.set()
        listener_thread.join(timeout=2)

        # Verify newvals reflects EXECUTE (index 3)
        self.assertTrue(listener_module.newvals[3],
            "Execute should be True (index 3)")

    def test_listener_state_transition_sequence(self):
        """
        Test that listener correctly tracks a sequence of state changes.
        This simulates a typical PackML startup sequence:
        STOPPED -> IDLE -> EXECUTE
        """
        stop_event = threading.Event()

        # Start listener
        listener_thread = threading.Thread(
            target=plc_listener,
            args=(stop_event, MOCK_PLC_ENDPOINT)
        )
        listener_thread.start()

        try:
            # Step 1: STOPPED
            self.plc.set_state('Stopped')
            time.sleep(0.2)
            self.assertTrue(listener_module.newvals[0], "Should be in STOPPED")

            # Step 2: IDLE (after Reset command in real system)
            self.plc.set_state('Idle')
            time.sleep(0.2)
            self.assertTrue(listener_module.newvals[1], "Should be in IDLE")
            self.assertFalse(listener_module.newvals[0], "Should NOT be in STOPPED")

            # Step 3: EXECUTE (after Start command in real system)
            self.plc.set_state('Execute')
            time.sleep(0.2)
            self.assertTrue(listener_module.newvals[3], "Should be in EXECUTE")
            self.assertFalse(listener_module.newvals[1], "Should NOT be in IDLE")
        finally:
            stop_event.set()
            listener_thread.join(timeout=2)

    def test_only_one_state_active(self):
        """
        Only ONE state may be active at any time.
        Test that the mock PLC and listener maintain this invariant.
        """
        stop_event = threading.Event()
        listener_thread = threading.Thread(
            target=plc_listener,
            args=(stop_event, MOCK_PLC_ENDPOINT)
        )
        listener_thread.start()

        try:
            for state in ['Stopped', 'Idle', 'Execute', 'Held', 'Suspended', 'Aborted']:
                self.plc.set_state(state)
                time.sleep(0.2)
                active_count = sum(1 for v in listener_module.newvals if v)
                self.assertEqual(active_count, 1,
                    f"Only ONE state should be active when in {state}, got {active_count}")
        finally:
            stop_event.set()
            listener_thread.join(timeout=2)


class TestDriverListenerSendData(unittest.TestCase):
    """Test DriverListener.send_data populates AllStatus response correctly."""

    def setUp(self):
        rclpy.init(args=None)
        self.driver = DriverListener()

    def tearDown(self):
        rclpy.shutdown()

    def test_send_data_stopped(self):
        """Test send_data returns correct response for STOPPED state."""
        listener_module.newvals = [True] + [False] * 16
        res = AllStatus.Response()
        self.driver.send_data(None, res)

        self.assertTrue(res.stopped_state)
        self.assertFalse(res.idle_state)
        self.assertFalse(res.execute_state)

    def test_send_data_execute(self):
        """Test send_data returns correct response for EXECUTE state."""
        listener_module.newvals = [False, False, False, True] + [False] * 13
        res = AllStatus.Response()
        self.driver.send_data(None, res)

        self.assertFalse(res.stopped_state)
        self.assertTrue(res.execute_state)

    def test_send_data_all_17_states(self):
        """Test send_data maps all 17 state indices correctly."""
        for idx, state in enumerate(PACKML_STATE_NAMES):
            state_array = [False] * 17
            state_array[idx] = True
            listener_module.newvals = state_array

            res = AllStatus.Response()
            self.driver.send_data(None, res)

            # Check at least the main states
            if idx == 0:
                self.assertTrue(res.stopped_state, f"Index {idx} should set stopped_state")
            elif idx == 1:
                self.assertTrue(res.idle_state, f"Index {idx} should set idle_state")
            elif idx == 3:
                self.assertTrue(res.execute_state, f"Index {idx} should set execute_state")


class TestTimeCounters(unittest.TestCase):
    """Test that time counters increment correctly while in each state."""

    @classmethod
    def setUpClass(cls):
        """Start the mock PLC server once for all tests in this class."""
        cls.plc = PackMLMockPLC(endpoint=MOCK_PLC_ENDPOINT)
        cls.plc.start()
        time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        """Stop the mock PLC server."""
        cls.plc.stop()
        time.sleep(0.3)

    def test_time_stopped_increments(self):
        """Test that time_stopped increments while in STOPPED state."""
        self.plc.set_state('Stopped')

        # Reset time counter
        listener_module.time_stopped = 0.0

        stop_event = threading.Event()
        listener_thread = threading.Thread(
            target=plc_listener,
            args=(stop_event, MOCK_PLC_ENDPOINT)
        )
        listener_thread.start()
        time.sleep(0.5)  # Allow 5 poll cycles (0.1s each)
        stop_event.set()
        listener_thread.join(timeout=2)

        # Time should have incremented (~0.5s)
        self.assertGreater(listener_module.time_stopped, 0.3,
            f"time_stopped should increment, got {listener_module.time_stopped}")

    def test_time_execute_increments(self):
        """Test that time_execute increments while in EXECUTE state."""
        self.plc.set_state('Execute')

        # Reset time counter
        listener_module.time_execute = 0.0

        stop_event = threading.Event()
        listener_thread = threading.Thread(
            target=plc_listener,
            args=(stop_event, MOCK_PLC_ENDPOINT)
        )
        listener_thread.start()
        time.sleep(0.5)
        stop_event.set()
        listener_thread.join(timeout=2)

        self.assertGreater(listener_module.time_execute, 0.3,
            f"time_execute should increment, got {listener_module.time_execute}")

    def test_time_counter_in_response(self):
        """Test that send_data returns accumulated time counters."""
        # Setup: run listener to accumulate time
        self.plc.set_state('Idle')
        listener_module.time_idle = 0.0

        stop_event = threading.Event()
        listener_thread = threading.Thread(
            target=plc_listener,
            args=(stop_event, MOCK_PLC_ENDPOINT)
        )
        listener_thread.start()
        time.sleep(0.5)
        stop_event.set()
        listener_thread.join(timeout=2)

        # Now check send_data returns the time
        import rclpy
        rclpy.init(args=None)
        try:
            driver = DriverListener()
            res = AllStatus.Response()
            driver.send_data(None, res)

            self.assertGreater(res.t_idle_state, 0.3,
                f"t_idle_state should reflect accumulated time, got {res.t_idle_state}")
        finally:
            rclpy.shutdown()


class TestSenderListenerIntegration(unittest.TestCase):
    """Integration tests: sender sends command, listener sees state change."""

    @classmethod
    def setUpClass(cls):
        import rclpy
        rclpy.init(args=None)
        cls.plc = PackMLMockPLC(endpoint=MOCK_PLC_ENDPOINT)
        cls.plc.start()
        time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        import rclpy
        cls.plc.stop()
        time.sleep(0.3)
        rclpy.shutdown()

    def test_sender_command_listener_detects_state(self):
        """
        Full integration: Sender sends RESET, listener detects IDLE state.
        This tests the complete data path: Sender -> OPC UA -> Mock PLC -> Listener.
        """
        from packml_plc.packml_plc_sender import DriverSender
        from packml_msgs.srv import StateChange

        # Reset to known state
        self.plc.set_state('Stopped')

        # Start listener
        stop_event = threading.Event()
        listener_thread = threading.Thread(
            target=plc_listener,
            args=(stop_event, MOCK_PLC_ENDPOINT)
        )
        listener_thread.start()
        time.sleep(0.3)  # Let listener start

        try:
            # Verify listener sees STOPPED
            self.assertTrue(listener_module.newvals[0],
                "Listener should initially see STOPPED")

            # Connect sender and send RESET command
            sender = DriverSender(endpoint=MOCK_PLC_ENDPOINT)
            sender.client.connect()

            req = StateChange.Request()
            res = StateChange.Response()
            req.command = StateChange.Request.RESET
            sender.trans_request(req, res)

            self.assertTrue(res.success, "RESET command should succeed")

            time.sleep(0.3)  # Let listener poll the new state

            # Verify listener now sees IDLE
            self.assertTrue(listener_module.newvals[1],
                "Listener should now see IDLE after RESET")
            self.assertFalse(listener_module.newvals[0],
                "Listener should no longer see STOPPED")

            sender.client.disconnect()
        finally:
            stop_event.set()
            listener_thread.join(timeout=2)

    def test_full_startup_sequence_through_listener(self):
        """
        Complete startup sequence: STOPPED -> IDLE -> EXECUTE
        Verified through listener state array.
        """
        from packml_plc.packml_plc_sender import DriverSender
        from packml_msgs.srv import StateChange

        # Reset to STOPPED
        self.plc.set_state('Stopped')

        # Start listener
        stop_event = threading.Event()
        listener_thread = threading.Thread(
            target=plc_listener,
            args=(stop_event, MOCK_PLC_ENDPOINT)
        )
        listener_thread.start()
        time.sleep(0.3)

        try:
            sender = DriverSender(endpoint=MOCK_PLC_ENDPOINT)
            sender.client.connect()

            # Step 1: RESET (STOPPED -> IDLE)
            req = StateChange.Request()
            res = StateChange.Response()
            req.command = StateChange.Request.RESET
            sender.trans_request(req, res)
            time.sleep(0.3)

            self.assertTrue(listener_module.newvals[1],
                "After RESET: listener should see IDLE")

            # Step 2: START (IDLE -> EXECUTE)
            req.command = StateChange.Request.START
            sender.trans_request(req, res)
            time.sleep(0.3)

            self.assertTrue(listener_module.newvals[3],
                "After START: listener should see EXECUTE")

            # Step 3: STOP (EXECUTE -> STOPPED)
            req.command = StateChange.Request.STOP
            sender.trans_request(req, res)
            time.sleep(0.3)

            self.assertTrue(listener_module.newvals[0],
                "After STOP: listener should see STOPPED")

            sender.client.disconnect()
        finally:
            stop_event.set()
            listener_thread.join(timeout=2)


if __name__ == '__main__':
    unittest.main()
