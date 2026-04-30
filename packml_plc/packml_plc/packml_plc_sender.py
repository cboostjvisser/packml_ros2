#!/usr/bin/env python
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
# Contributors: Dejanira Araiza Illan, Derrick Ang Ming Yan
#

import time

from opcua import Client, ua
from packml_msgs.srv import StateChange
import rclpy
from rclpy.node import Node


class DriverSender(Node):
    """
    This class sends PackML state change commands from ROS2 to the PLC via OPC UA.
    Each command is mapped to a specific OPC UA node, following the command names/order defined in the packml_sm library.
    """

    def __init__(self, endpoint=None):
        super().__init__('driver_sender')
        # TODO: choose; ROS Parameter or function parameter
        self.declare_parameter('opcua_endpoint', 'opc.tcp://127.0.0.1:4840/freeopcua/server/')
        if endpoint is None:
            endpoint = self.get_parameter('opcua_endpoint').get_parameter_value().string_value
        self.srv = self.create_service(StateChange, 'transition', self.trans_request)
        self.client = Client(endpoint)

    def connect(self):
        self.client.connect()

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.client.disconnect()

    def trans_request(self, req, res):
        """
        Receives a state change request from ROS2 and writes the corresponding PackML command to the PLC.
        Sends a short True pulse on the command tag, then resets it to False.
        """
        command_rtn = False
        command_valid = True
        command_int = req.command
        # Mapping from StateChange.Request enum to OPC UA node names (packml_sm order)
        command_map = {
            StateChange.Request.ABORT:      'Cmd_Abort',
            StateChange.Request.CLEAR:      'Cmd_Clear',
            StateChange.Request.HOLD:       'Cmd_Hold',
            StateChange.Request.RESET:      'Cmd_Reset',
            StateChange.Request.START:      'Cmd_Start',
            StateChange.Request.STOP:       'Cmd_Stop',
            StateChange.Request.SUSPEND:    'Cmd_Suspend',
            StateChange.Request.UNHOLD:     'Cmd_Unhold',
            StateChange.Request.UNSUSPEND:  'Cmd_Unsuspend',
        }
        try:
            if command_int in command_map:
                cmd_name = command_map[command_int]
                # Original node: 'ns=3;' + 's=\"PackML_Status\".\"EM00\"' + '.\"Unit\".\"Cmd_Abort\"'
                node = self.client.get_node(f'ns=3;s="PackML_Status"."EM00"."Unit"."{cmd_name}"')

                # Write a short pulse so PLC logic can consume edge-like command semantics.
                result = node.set_attribute(ua.AttributeIds.Value, ua.DataValue(True))
                self.get_logger().info(f'Sending {cmd_name} Command, result={result}')
                time.sleep(0.1)
                node.set_attribute(ua.AttributeIds.Value, ua.DataValue(False))
                command_rtn = True
            else:
                self.get_logger().error(f"Unsupported or unrecognized command: {command_int}")
                command_valid = False
        except Exception as ex:
            self.get_logger().error(f"Exception while sending command: {ex}")
            command_valid = False
        if command_valid:
            if command_rtn:
                self.get_logger().info(f'Successful transition request command: {command_int}')
                res.success = True
                res.error_code = res.SUCCESS
            else:
                res.success = False
                res.error_code = res.INVALID_TRANSITION_REQUEST
        else:
            res.success = False
            res.error_code = res.UNRECOGNIZED_REQUEST
        return res


def main(args=None):
    rclpy.init(args=args)
    driver_sender = DriverSender()
    driver_sender.connect()
    rclpy.spin(driver_sender)


if __name__ == '__main__':
    main()
