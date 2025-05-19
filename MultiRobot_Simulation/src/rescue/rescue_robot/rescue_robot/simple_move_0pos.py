import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time

class ArmPositionController(Node):
    def __init__(self):
        super().__init__('arm_position_controller')
        #self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        time.sleep(2)  # Ensure the publisher is fully set up
        self.move_joints([0.7, 0.5])

    def move_joints(self, positions):
        msg = JointTrajectory()
        msg.joint_names = ['arm_base_forearm_joint', 'forearm_hand_joint']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2  # Move in 2 seconds
        
        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Moving joints to positions: {positions}')


def main(args=None):
    rclpy.init(args=args)
    controller = ArmPositionController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
