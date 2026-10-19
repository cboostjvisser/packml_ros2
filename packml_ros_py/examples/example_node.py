#!/usr/bin/env python3
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

"""Example PackML Python node that can be managed by the PackML manager."""

import threading
import time

import rclpy

from packml_ros_py import PackmlNode, State


class ExamplePackmlNode(PackmlNode):
    """A simple PackML equipment module implemented in Python."""

    def __init__(self):
        super().__init__('example_packml_py_node')

    def defers_completion(self, state: State) -> bool:
        # Only RESETTING defers -- every other coordinated state completes the
        # instant on_state_transition_request() approves it (the default). See
        # on_deferred_work() below for the work itself.
        return state == State.RESETTING

    def on_state_transition_request(self, target_state: State) -> bool:
        self.get_logger().info(f'[Example] State transition to {target_state.name} requested')
        return True

    def on_deferred_work(self, state: State, completion) -> None:
        # Simulate real commanded work (e.g. homing an axis) that finishes some time after the
        # transition is accepted, then report it through this goal's own handle -- so it stays
        # this goal's completion even if a later RESETTING starts in the meantime.
        threading.Thread(
            target=self._finish_resetting, args=(completion,), daemon=True).start()

    def _finish_resetting(self, completion) -> None:
        time.sleep(0.5)
        completion.report(True)

    def on_mode_transition_request(self, target_mode: int) -> bool:
        self.get_logger().info(f'[Example] Mode transition to {target_mode} requested')
        # Accept all mode changes
        return True

    def on_packml_status_changed(self, status) -> None:
        self.get_logger().info(
            f'[Example] Status changed - State: {State(status.state.val).name}, '
            f'Mode: {status.mode.val}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ExamplePackmlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
