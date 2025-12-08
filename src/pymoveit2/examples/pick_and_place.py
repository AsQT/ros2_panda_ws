#!/usr/bin/env python3
"""
Pick and place node combining Cartesian and joint-space moves.
Fixed: IndentationError and Start State Invalid.
"""

from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import math

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_color", "R")
        self.target_color = self.get_parameter("target_color").value.upper()

        # Flags
        self.already_moved = False
        self.target_coords = None

        self.callback_group = ReentrantCallbackGroup()

        # --- FIX: Force robot to valid pose (Joint 4 = -90 deg) ---
        self.traj_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.get_logger().info("Dang cuong ep robot ve vi tri hop le (Joint 4 = -90 do)...")
        self.force_move_to_valid_pose()
        time.sleep(3.0) 
        self.get_logger().info("Robot da vao vi tri an toan. Khoi dong MoveIt...")
        # --------------------------------------------------------

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        # Set lower velocity & acceleration for smoother motion
        self.moveit2.max_velocity = 0.3
        self.moveit2.max_acceleration = 0.3

        # Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Subscriber
        self.sub = self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10
        )
        self.get_logger().info(f"Waiting for {self.target_color} from /color_coordinates...")

        # Predefined joint positions (in radians) - Joint 4 must be negative
        self.start_joints = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, 0.0, math.radians(45.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        self.drop_joints  = [math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
                             math.radians(-124.0), math.radians(44.0), math.radians(163.0), math.radians(7.0)]

        # Move to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

    def force_move_to_valid_pose(self):
        msg = JointTrajectory()
        msg.joint_names = panda.joint_names()
        point = JointTrajectoryPoint()
        # Safe pose: Joint 4 bent to -1.57
        point.positions = [0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.785]
        point.time_from_start = Duration(sec=2)
        msg.points.append(point)
        self.traj_pub.publish(msg)

    def coords_callback(self, msg):
        if self.already_moved:
            return

        try:
            parts = msg.data.split(",")
            # Handle cases where msg format might vary slightly
            if len(parts) >= 4:
                color_id = parts[0].strip().upper()
                x, y, z = parts[1], parts[2], parts[3]
            else:
                return

            if color_id == self.target_color:
                self.target_coords = [float(x), float(y), float(z)]
                self.get_logger().info(
                    f"Target {self.target_color} locked at: "
                    f"[{self.target_coords[0]:.3f}, {self.target_coords[1]:.3f}, {self.target_coords[2]:.3f}]"
                )
                self.already_moved = True

                # --- FIX: Z=0.5m (Hardcoded) ---
                pick_position = [self.target_coords[0], self.target_coords[1], 0.5]
                
                # Gripper orientation (Standard top-down for Panda)
                quat_xyzw = [1.0, 0.0, 0.0, 0.0] 

                # --- Sequence ---
                self.get_logger().info("1. Moving to HOME...")
                self.moveit2.move_to_configuration(self.home_joints)
                self.moveit2.wait_until_executed()

                self.get_logger().info("2. Moving to PICK (High)...")
                self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
                self.moveit2.wait_until_executed()

                self.get_logger().info("3. Opening Gripper...")
                self.gripper.open()
                self.gripper.wait_until_executed()

                self.get_logger().info("4. Approaching (Down)...")
                approach_position = [pick_position[0], pick_position[1], pick_position[2] - 0.31]
                self.moveit2.move_to_pose(position=approach_position, quat_xyzw=quat_xyzw)
                self.moveit2.wait_until_executed()

                self.get_logger().info("5. Closing Gripper...")
                self.gripper.close()
                self.gripper.wait_until_executed()

                self.get_logger().info("6. Lifting UP...")
                self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
                self.moveit2.wait_until_executed()

                self.get_logger().info("7. Moving to HOME...")
                self.moveit2.move_to_configuration(self.home_joints)
                self.moveit2.wait_until_executed()

                self.get_logger().info("8. Moving to DROP...")
                self.moveit2.move_to_configuration(self.drop_joints)
                self.moveit2.wait_until_executed()

                self.get_logger().info("9. Releasing...")
                self.gripper.open()
                self.gripper.wait_until_executed()

                self.get_logger().info("Done!")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error parsing coordinates: {e}")

def main():
    rclpy.init()
    node = PickAndPlace()
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
