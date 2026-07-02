# Launch file for PackML Health System Demo
#
# Demonstrates:
#   - Three Equipment Modules (motor, sensor, conveyor) each publishing heartbeats
#   - PackML manager with embedded HealthMonitor monitoring all three
#   - Fault injection via `ros2 param set` at runtime
#   - RViz panel showing state machine transitions
#
# Usage:
#   ros2 launch packml_ros2 packml_health_demo.launch.py
#
# Inject faults at runtime (in a separate terminal):
#   ros2 param set /demo_motor_driver  inject_fault abort    # triggers ABORT
#   ros2 param set /demo_motor_driver  inject_fault hold     # triggers HOLD
#   ros2 param set /demo_sensor_hub    inject_fault sensor   # triggers HOLD
#   ros2 param set /demo_sensor_hub    inject_fault range    # triggers SUSPEND
#   ros2 param set /demo_conveyor      inject_fault jam      # triggers HOLD
#   ros2 param set /demo_conveyor      inject_fault starved  # triggers SUSPEND
#   ros2 param set /demo_motor_driver  inject_fault none     # clear fault
#
# Watch heartbeats:
#   ros2 topic echo /demo_motor_driver/heartbeat
#   ros2 topic echo /demo_sensor_hub/heartbeat
#   ros2 topic echo /demo_conveyor/heartbeat
#
# Drive the state machine manually (after all nodes are healthy):
#   ros2 service call /packml_ros_node/changeState packml_msgs/srv/StateChange "{command: 1}"  # RESET
#   ros2 service call /packml_ros_node/changeState packml_msgs/srv/StateChange "{command: 2}"  # START
#   (command ints: RESET=1 START=2 STOP=3 HOLD=4 UNHOLD=5 SUSPEND=6 UNSUSPEND=7 ABORT=8 CLEAR=9)
#
# Expected demo flow:
#   1. All three Equipment Modules start → HealthMonitor sees all healthy
#   2. Manager gate opens → RESET → START → EXECUTE
#   3. Inject motor abort → machine transitions to ABORTING → ABORTED
#   4. Clear fault → machine waits for operator CLEAR command
#   5. CLEAR → STOPPED → RESET → EXECUTE (machine recovers)
#   6. Inject sensor "range" → machine SUSPENDS (external condition)
#   7. Clear sensor fault → machine waits for UNSUSPEND command

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo, TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('packml_ros2'),
        'rviz',
        'packml_ros_demo_panel.rviz'
    )

    return LaunchDescription([

        # ── Equipment Module: Motor Driver ──────────────────────────────
        Node(
            package='packml_ros',
            executable='example_health_demo',
            name='demo_motor_driver',
            arguments=['motor'],
            parameters=[{
                'heartbeat_interval_ms': 1000,
                'inject_fault': 'none',
            }],
            output='screen',
            emulate_tty=True,
        ),

        # ── Equipment Module: Sensor Hub ─────────────────────────────────
        Node(
            package='packml_ros',
            executable='example_health_demo',
            name='demo_sensor_hub',
            arguments=['sensor'],
            parameters=[{
                'heartbeat_interval_ms': 500,
                'inject_fault': 'none',
            }],
            output='screen',
            emulate_tty=True,
        ),

        # ── Equipment Module: Conveyor ───────────────────────────────────
        Node(
            package='packml_ros',
            executable='example_health_demo',
            name='demo_conveyor',
            arguments=['conveyor'],
            parameters=[{
                'heartbeat_interval_ms': 1000,
                'inject_fault': 'none',
            }],
            output='screen',
            emulate_tty=True,
        ),

        # ── PackML Manager ───────────────────────────────────────────────
        # The manager's embedded HealthMonitor subscribes to each node's
        # heartbeat topic and enforces the health gate.
        #
        # required_nodes: transitions from STOPPED blocked until all healthy.
        # heartbeat_timeout_factor: 3 × node's interval = timeout per node.
        Node(
            package='packml_ros',
            executable='packml_ros_node',
            name='packml_ros_node',
            parameters=[{
                'required_nodes': [
                    'demo_motor_driver',
                    'demo_sensor_hub',
                    'demo_conveyor',
                ],
                'heartbeat_timeout_factor': 3.0,
                'heartbeat_startup_grace_ms': 10000,
                'manual_mode_allows_health_bypass': False,
            }],
            arguments=['--ros-args', '--log-level', 'packml_ros:=debug'],
            output='screen',
            emulate_tty=True,
        ),
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen',
        ),

        # ── Startup info ─────────────────────────────────────────────────
        TimerAction(
            period=2.0,
            actions=[
                LogInfo(msg=(
                    '\n'
                    '========================================\n'
                    '  PackML Health Demo is running.\n'
                    '\n'
                    '  Equipment Modules:\n'
                    '    /demo_motor_driver  (heartbeat: 1000ms)\n'
                    '    /demo_sensor_hub    (heartbeat:  500ms)\n'
                    '    /demo_conveyor      (heartbeat: 1000ms)\n'
                    '\n'
                    '  Inject a fault:\n'
                    '    ros2 param set /demo_motor_driver inject_fault abort\n'
                    '    ros2 param set /demo_motor_driver inject_fault none\n'
                    '\n'
                    '  Drive the machine (once all nodes healthy):\n'
                    '    ros2 service call /packml_ros_node/changeState '
                    'packml_msgs/srv/StateChange "{command: 1}"  # RESET\n'
                    '    ros2 service call /packml_ros_node/changeState '
                    'packml_msgs/srv/StateChange "{command: 2}"  # START\n'
                    '========================================\n'
                )),
            ],
        ),
    ])
