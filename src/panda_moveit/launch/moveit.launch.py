import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    panda_description_pkg = get_package_share_directory("panda_description")
    panda_moveit_pkg = get_package_share_directory("panda_moveit")

    rviz_config = os.path.join(panda_moveit_pkg, "rviz", "moveit.rviz")

    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit")
        .robot_description(
            file_path=os.path.join(
                panda_description_pkg,
                "urdf",
                "panda.urdf.xacro",
            ),
            mappings={
                # MoveIt thuần: không tự quyết định backend control
                "is_ignition": "false",
                # Ở đây để true để robot_description hợp với mode không-Gazebo.
                # Khi chạy với Gazebo, backend thật nằm ở terminal Gazebo.
                "use_mock_hardware": "true",
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            {"publish_robot_description_semantic": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time when running with Gazebo",
            ),
            move_group_node,
            rviz_node,
        ]
    )