import os
import launch
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    package_directory = get_package_share_directory("lilbot_webot_sim")
    core_directory = get_package_share_directory("webots_ros2_core")

    webots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_directory, "launch", "robot_launch.py")
        ),
        launch_arguments=[
            ("package", "lilbot_webot_sim"),
            ("executable", "enable_robot"),
            ("name", "lilbot"),
            ("world", PathJoinSubstitution([package_directory, "worlds", "lilbot_world.wbt"])),
        ]
    )

    rviz_path = package_directory+"/rviz/config.rviz"
    rviz2 = Node(
        package="rviz2", 
        executable="rviz2", 
        name="rviz2", 
        output="screen", 
        arguments=["-d"+str(rviz_path)]
    )

    lilbot_driver = Node(
        package='lilbot_webot_sim',
        executable='line_follower',
        name='master_node'
    )

    return LaunchDescription([
        webots,
        rviz2,
        lilbot_driver,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=webots,
        #         on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        #     )
        # )
    ])