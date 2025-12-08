import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class FixPoseNode(Node):
    def __init__(self):
        super().__init__('fix_pose_node')
        # Topic này gửi lệnh trực tiếp xuống controller
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.get_logger().info('Dang chuan bi dua robot ve vi tri an toan...')

    def timer_callback(self):
        if self.count < 3: # Gửi lệnh 3 lần cho chắc ăn
            msg = JointTrajectory()
            msg.joint_names = [
                'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4',
                'panda_joint5', 'panda_joint6', 'panda_joint7'
            ]
            
            point = JointTrajectoryPoint()
            # Gán Joint 4 = -1.57 (Gập khuỷu tay 90 độ)
            point.positions = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785]
            point.time_from_start = Duration(sec=2) # Di chuyển trong 2 giây
            
            msg.points.append(point)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Da gui lenh sua loi lan {self.count + 1}/3...')
            self.count += 1
        else:
            self.get_logger().info('Xong! Robot da vao vi tri. Ban co the chay MoveIt ngay bay gio.')
            exit()

def main(args=None):
    rclpy.init(args=args)
    node = FixPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
