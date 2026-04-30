from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the absolute path to the RViz config in the packml_plugin package
    rviz_config = os.path.join(
        get_package_share_directory('packml_plugin'),
        'rviz',
        'packml_panel_only.rviz'
    )
    rviz_args = ["-d", rviz_config]

    from launch.actions import TimerAction

    return LaunchDescription([
        # Launch the mock PLC OPC UA server from the installed share directory
        ExecuteProcess(
            cmd=["python3", os.path.join(get_package_share_directory('packml_plc'), "packml_mock_plc.py")],
            output="screen"
        ),
        # Delay sender and listener startup to ensure mock PLC is ready
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="packml_plc",
                    executable="packml_plc_sender",
                    name="packml_plc_sender",
                    output="screen"
                ),
                Node(
                    package="packml_plc",
                    executable="packml_plc_listener",
                    name="packml_plc_listener",
                    output="screen"
                ),
            ]
        ),
        # Launch RViz with the packml_plugin panel loaded
        ExecuteProcess(
            cmd=["rviz2"] + rviz_args,
            output="screen"
        )
    ])
