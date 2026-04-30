# Launch file for PackML ROS state machine demo
# This demo launches the RViz panel and the packml_ros_node, which runs the PackML state machine.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch the PackML ROS state machine demo:
    - Starts RViz with the PackML panel
    - Starts the packml_ros_node (runs the state machine)
    """

    rviz_config = os.path.join(
        get_package_share_directory('packml_ros2'),
        'rviz',
        'packml_ros_demo_panel.rviz'
    )
    rviz_args = ["-d", rviz_config]

    return LaunchDescription([
        # Launch RViz with the packml_plugin panel loaded
        ExecuteProcess(
            cmd=["rviz2"] + rviz_args,
            output="screen"
        ),
        # Launch the PackML ROS state machine node
        Node(
            package="packml_ros",
            executable="packml_ros_node",
            name="packml_ros_node",
            output="screen"
        )
    ])
