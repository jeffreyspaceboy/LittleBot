import os
import pathlib
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_directory = get_package_share_directory("lilbot_driver")

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="lilbot_driver",
            executable="lilbot_driver_node",
            output="screen"
        )
    ])