import os
from os import pathsep
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    panda_description = get_package_share_directory("panda_description")
    panda_controller = get_package_share_directory("panda_controller")

    model = LaunchConfiguration("model")
    world_name = LaunchConfiguration("world_name")
    is_sim = LaunchConfiguration("is_sim")
    is_ignition = LaunchConfiguration("is_ignition")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(panda_description, "urdf", "panda.urdf.xacro"),
        description="Absolute path to robot xacro file",
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="workspace",
        description="World name without extension",
    )

    is_sim_arg = DeclareLaunchArgument(
        name="is_sim",
        default_value="true",
        description="Use simulation time",
    )

    # Theo logic xacro hiện tại của bạn:
    #   is_ignition=false  -> dùng gz_ros2_control
    # nên ở Jazzy/ros_gz_sim ta để mặc định false
    is_ignition_arg = DeclareLaunchArgument(
        name="is_ignition",
        default_value="false",
        description="Compatibility switch for ignition/gz branch in xacro",
    )

    use_mock_hardware_arg = DeclareLaunchArgument(
        name="use_mock_hardware",
        default_value="false",
        description="Must be false when running with Gazebo backend",
    )

    world_path = PathJoinSubstitution(
        [
            panda_description,
            "worlds",
            PythonExpression(["'", world_name, "'", " + '.world'"]),
        ]
    )

    model_path = str(Path(panda_description).parent.resolve())
    model_path += pathsep + os.path.join(panda_description, "models")

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path,
    )

    controllers_yaml = os.path.join(
        panda_controller,
        "config",
        "panda_controllers.yaml",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro",
                " ",
                model,
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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_path, " -v 4 -r'"])
        }.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "panda",
            "-x", "-0.2",
            "-y", "0.0",
            "-z", "0.285",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0",
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            "/astra/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/astra/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",

            "/astra/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/astra/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/astra/depth/image_raw/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        output="screen",
        )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
        output="screen",
    )

    # controller_manager trong Gazebo sẽ do plugin gz_ros2_control / ign_ros2_control tạo ra
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
            "20",
            "--switch-timeout",
            "20",
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
            "20",
            "--switch-timeout",
            "20",
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
            "20",
            "--switch-timeout",
            "20",
        ],
    )

    start_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
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
            model_arg,
            world_name_arg,
            is_sim_arg,
            is_ignition_arg,
            use_mock_hardware_arg,
            gazebo_resource_path,
            robot_state_publisher_node,
            gazebo,
            gz_spawn_entity,
            start_jsb_after_spawn,
            start_arm_after_jsb,
            start_gripper_after_arm,
            gz_ros2_bridge,
            ros_gz_image_bridge,
        ]
    )