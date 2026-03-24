from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="panda_gui",
            executable="panda_gui",
            name="panda_gui",
            output="screen",
            parameters=[
                {"joint_names": ["panda_joint1",
                                 "panda_joint2",
                                 "panda_joint3",
                                 "panda_joint4",
                                 "panda_joint5",
                                 "panda_joint6"
                                 "panda_joint7"]},
                {"axis_ids": [0,1,2,3,4,5,6]},
                {"group_name": "arm"},
                {"base_frame": "panda_link0"},
                {"ee_link": "tcp_link"},
            ],
        )
    ])
