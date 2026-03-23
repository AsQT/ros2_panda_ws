import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    is_sim = LaunchConfiguration("is_sim")
    is_ignition = LaunchConfiguration("is_ignition")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    panda_description_pkg = get_package_share_directory("panda_description")
    panda_controller_pkg = get_package_share_directory("panda_controller")

    xacro_file = os.path.join(
        panda_description_pkg,
        "urdf",
        "panda.urdf.xacro",
    )

    controllers_yaml = os.path.join(
        panda_controller_pkg,
        "config",
        "panda_controllers.yaml",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                xacro_file,
                " ",
                "is_sim:=",
                is_sim,
                " ",
                "is_ignition:=",
                is_ignition,
                " ",
                "use_mock_hardware:=",
                use_mock_hardware,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": is_sim,
            }
        ],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": is_sim,
                # mock mode cho phép controller active dễ hơn
                "defaults.allow_controller_activation_with_inactive_hardware": True,
            },
            controllers_yaml,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_yaml,
            "--controller-manager-timeout",
            "15",
            "--switch-timeout",
            "15",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_yaml,
            "--controller-manager-timeout",
            "15",
            "--switch-timeout",
            "15",
        ],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "gripper_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_yaml,
            "--controller-manager-timeout",
            "15",
            "--switch-timeout",
            "15",
        ],
    )

    start_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    start_gripper_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("is_sim", default_value="false"),
            DeclareLaunchArgument("is_ignition", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="true"),
            robot_state_publisher_node,
            controller_manager_node,
            joint_state_broadcaster_spawner,
            start_arm_after_jsb,
            start_gripper_after_arm,
        ]
    )