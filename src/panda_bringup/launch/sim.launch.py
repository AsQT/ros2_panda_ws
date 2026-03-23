import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    target_color = LaunchConfiguration("target_color")

    panda_description_pkg = get_package_share_directory("panda_description")
    panda_moveit_pkg = get_package_share_directory("panda_moveit")

    # 1) Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                panda_description_pkg,
                "launch",
                "gazebo.launch.py",
            )
        )
    )

    # 2) MoveIt 
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                panda_moveit_pkg,
                "launch",
                "moveit.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            gazebo,

            # Đợi Gazebo spawn robot + controller rồi mới mở MoveIt
            TimerAction(
                period=4.0,
                actions=[moveit],
            ),

        ]
    )