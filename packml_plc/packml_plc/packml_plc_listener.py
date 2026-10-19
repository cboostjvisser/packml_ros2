#!/usr/bin/env python3
#
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
# Contributors: Dejanira Araiza Illan, Derrick Ang Ming Yan
#

import threading
import time

from opcua import Client
from packml_msgs.srv import AllStatus
import rclpy
from rclpy.node import Node

newvals = [False, False, False, False, False, False, False, False,
           False, False, False, False, False, False, False, False, False]
time_stopped = 0.0
time_idle = 0.0
time_starting = 0.0
time_execute = 0.0
time_completing = 0.0
time_complete = 0.0
time_clearing = 0.0
time_suspended = 0.0
time_aborting = 0.0
time_aborted = 0.0
time_holding = 0.0
time_held = 0.0
time_unholding = 0.0
time_suspending = 0.0
time_unsuspending = 0.0
time_resetting = 0.0
time_stopping = 0.0
thee = threading.Event()


class HelloClient:
    """This class creates an OPCUA client to connect to the PLC server."""

    def __init__(self, endpoint=None):
        if endpoint is None:
            endpoint = 'opc.tcp://127.0.0.1:4840/freeopcua/server/'
        self.client = Client(endpoint)

    def __enter__(self):
        self.client.connect()
        return self.client

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.client.disconnect()


class DriverListener(Node):

    """
    This class mirrors the PLC's PackML state in ROS2 (PLC Master, ROS2 Slave).
    It reads the PLC's PackML state tags via OPC UA and updates the ROS2 node state accordingly.
    Only one state should be active at a time, matching the state order and logic defined in the packml_sm library.
    """

    def __init__(self):
        super().__init__('driver_listener')
        self.declare_parameter('opcua_endpoint', 'opc.tcp://127.0.0.1:4840/freeopcua/server/')
        self.opcua_endpoint = self.get_parameter('opcua_endpoint').get_parameter_value().string_value
        self.srv = self.create_service(AllStatus, 'allStatus', self.send_data)

    def send_data(self, req, res):
        global time_stopped
        global time_idle
        global time_starting
        global time_execute
        global time_completing
        global time_complete
        global time_clearing
        global time_suspended
        global time_aborting
        global time_aborted
        global time_holding
        global time_held
        global time_unholding
        global time_suspending
        global time_unsuspending
        global time_resetting
        global time_stopping
        global newvals
        res.stopped_state = newvals[0]
        res.idle_state = newvals[1]
        res.starting_state = newvals[2]
        res.execute_state = newvals[3]
        res.completing_state = newvals[4]
        res.complete_state = newvals[5]
        res.clearing_state = newvals[6]
        res.suspended_state = newvals[7]
        res.aborting_state = newvals[8]
        res.aborted_state = newvals[9]
        res.holding_state = newvals[10]
        res.held_state = newvals[11]
        res.unholding_state = newvals[12]
        res.suspending_state = newvals[13]
        res.unsuspending_state = newvals[14]
        res.resetting_state = newvals[15]
        res.stopping_state = newvals[16]
        res.t_stopped_state = time_stopped
        res.t_idle_state = time_idle
        res.t_starting_state = time_starting
        res.t_execute_state = time_execute
        res.t_completing_state = time_completing
        res.t_complete_state = time_complete
        res.t_clearing_state = time_clearing
        res.t_suspended_state = time_suspended
        res.t_aborting_state = time_aborting
        res.t_aborted_state = time_aborted
        res.t_holding_state = time_holding
        res.t_held_state = time_held
        res.t_unholding_state = time_unholding
        res.t_suspending_state = time_suspending
        res.t_unsuspending_state = time_unsuspending
        res.t_resetting_state = time_resetting
        res.t_stopping_state = time_stopping
        return res

# Order must match packml_sm library and AllStatus message
PACKML_STATE_NAMES = [
    "Stopped", "Idle", "Starting", "Execute", "Completing", "Complete",
    "Clearing", "Suspended", "Aborting", "Aborted", "Holding", "Held",
    "Unholding", "Suspending", "Unsuspending", "Resetting", "Stopping"
]

# Default PLC endpoint
# TODO: This is stored from original version, but could be removed?
DEFAULT_PLC_ENDPOINT = 'opc.tcp://192.168.125.2:4840/freeopcua/server/'

def plc_listener(e, endpoint=None):
    """Create the connection with the PLC, monitors the state in its state machine.

    Args:
        e: threading.Event to signal shutdown
        endpoint: OPC UA endpoint URL (optional, for testing with mock PLC)
    """
    if endpoint is None:
        endpoint = DEFAULT_PLC_ENDPOINT

    # Reset time counters
    global time_stopped, time_idle, time_starting, time_execute, time_completing
    global time_complete, time_clearing, time_suspended, time_aborting, time_aborted
    global time_holding, time_held, time_unholding, time_suspending, time_unsuspending
    global time_resetting, time_stopping
    time_stopped = time_idle = time_starting = time_execute = time_completing = 0.0
    time_complete = time_clearing = time_suspended = time_aborting = time_aborted = 0.0
    time_holding = time_held = time_unholding = time_suspending = time_unsuspending = 0.0
    time_resetting = time_stopping = 0.0

    # Open a connection with the PLC
    import rclpy
    node = rclpy.logging.get_logger("packml_plc_listener")
    node.info(f"Attempting OPC UA connection to endpoint: {endpoint}")
    try:
        with HelloClient(endpoint) as client:
            node.info(f"Connected to OPC UA endpoint: {endpoint}")
            try:
                while not e.is_set():
                    node_values = []
                    for state in PACKML_STATE_NAMES:
                        try:
                            # Original ID: 'ns=3;s=\"PackML_Status\".\"Sts\".' + '\"State\".\"Suspended\"'
                            node_obj = client.get_node(
                                f'ns=3;s="PackML_Status"."EM00"."Unit"."{state}"'
                            )
                            value = node_obj.get_value()
                        except Exception as node_exc:
                            node.error(f"Error reading state '{state}': {node_exc}")
                            value = False
                        node_values.append(value)
                    global newvals
                    newvals = node_values
                    active_count = sum(1 for v in newvals if v)
                    if active_count > 1:
                        active_states = [
                            (PACKML_STATE_NAMES[i], v)
                            for i, v in enumerate(newvals) if v
                        ]
                        node.warn(f"Warning: Multiple active states: {active_states}")
                    elif active_count == 0:
                        node.warn("Warning: No active state detected!")
                    if newvals[0]:
                        time_stopped += 0.1
                    if newvals[1]:
                        time_idle += 0.1
                    if newvals[2]:
                        time_starting += 0.1
                    if newvals[3]:
                        time_execute += 0.1
                    if newvals[4]:
                        time_completing += 0.1
                    if newvals[5]:
                        time_complete += 0.1
                    if newvals[6]:
                        time_clearing += 0.1
                    if newvals[7]:
                        time_suspended += 0.1
                    if newvals[8]:
                        time_aborting += 0.1
                    if newvals[9]:
                        time_aborted += 0.1
                    if newvals[10]:
                        time_holding += 0.1
                    if newvals[11]:
                        time_held += 0.1
                    if newvals[12]:
                        time_unholding += 0.1
                    if newvals[13]:
                        time_suspending += 0.1
                    if newvals[14]:
                        time_unsuspending += 0.1
                    if newvals[15]:
                        time_resetting += 0.1
                    if newvals[16]:
                        time_stopping += 0.1
                    # TODO: Instead of sleeping 0.1 and updating 0.1, update the actual time delta's
                    time.sleep(0.1)
            except KeyboardInterrupt:
                node.info("KeyboardInterrupt received, shutting down listener thread.")
            except Exception as poll_exc:
                node.error(f"Exception in polling loop: {poll_exc}")
    except Exception as conn_exc:
        node.error(f"OPC UA connection failed: {conn_exc}")


def main(args=None):
    rclpy.init(args=args)
    driver_listener = DriverListener()
    # Pass the parameter value to the thread
    # TODO: We can use ros threading here
    listener_thread = threading.Thread(target=plc_listener, args=(thee, driver_listener.opcua_endpoint))
    listener_thread.start()
    try:
        rclpy.spin(driver_listener)
    finally:
        # Signal polling thread to exit and wait for it
        thee.set()
        listener_thread.join(timeout=2)


if __name__ == '__main__':
    main()
