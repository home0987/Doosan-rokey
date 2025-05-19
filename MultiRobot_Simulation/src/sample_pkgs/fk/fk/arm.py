#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration

class OpenManipulatorControl(Node):
    def __init__(self):
        super().__init__('open_manipulator_control')
        
        # Define joint names for OpenMANIPULATOR-X
        self.joint_names = [
            'joint1',
            'joint2',
            'joint3',
            'joint4',
            'arm'
        ]
        
        # Create action client
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action server...')
        self.trajectory_client.wait_for_server()
        self.get_logger().info('Action server connected!')

    def move_to_joint_positions(self, positions, duration=5.0):
        """Send joint positions to the robot"""
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names[:-1]  # Exclude gripper
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.accelerations = [0.0] * len(positions)
        point.time_from_start = Duration(seconds=duration).to_msg()
        
        trajectory_msg.points.append(point)
        
        # Create goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory_msg
        
        # Send goal
        self.get_logger().info(f'Sending goal: {positions}')
        self._send_goal(goal_msg)

    def _send_goal(self, goal_msg):
        """Send goal and handle result"""
        future = self.trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code == 0:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with error code: {result.error_code}')

def main():
    rclpy.init()
    node = OpenManipulatorControl()
    
    try:
        # Example positions (in radians)
        # Home position
        node.move_to_joint_positions([0.0, 0.0, 0.0, 0.0])
        
        # Forward position
        node.move_to_joint_positions([0.0, -1.0, 0.3, 0.7])
        
        # Side position
        node.move_to_joint_positions([1.57, -0.8, 0.5, 0.8])
        
        # Return to home
        node.move_to_joint_positions([0.0, 0.0, 0.0, 0.0])
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
