#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from moveit2.move_group import MoveGroup

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import sys


class OpenManipulatorControl(Node):
    def __init__(self):
        super().__init__('open_manipulator_control')

        # Initialize moveit_commander and node
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Instantiate move_group
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        
        # Get planning scene interface
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Get robot commander
        self.robot = moveit_commander.RobotCommander()
        
        # Print robot info
        self.print_robot_info()

    def print_robot_info(self):
        """Print basic information about the robot"""
        self.get_logger().info(f"Planning Frame: {self.arm_group.get_planning_frame()}")
        self.get_logger().info(f"End Effector Link: {self.arm_group.get_end_effector_link()}")
        self.get_logger().info(f"Available Planning Groups: {self.robot.get_group_names()}")

    def set_pose_target(self, x, y, z, w=1.0):
        """Set a pose target for inverse kinematics"""
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = w
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        self.arm_group.set_pose_target(pose_goal)
        
        # Plan and execute
        success = self.plan_and_execute()
        
        # Clear targets
        self.arm_group.clear_pose_targets()
        
        return success

    def plan_and_execute(self):
        """Plan and execute movement"""
        plan = self.arm_group.plan()
        
        if plan[0]:
            self.get_logger().info("Plan found, executing...")
            return self.arm_group.execute(plan[1], wait=True)
        else:
            self.get_logger().error("Planning failed!")
            return False

    def move_to_joint_state(self, joint_goal):
        """Move to a specific joint state"""
        self.arm_group.set_joint_value_target(joint_goal)
        return self.plan_and_execute()

    def move_gripper(self, width):
        """Move the gripper"""
        self.gripper_group.set_joint_value_target([width, width])
        return self.gripper_group.go(wait=True)

    def move_cartesian_path(self, waypoints):
        """Execute a cartesian path through given waypoints"""
        (plan, fraction) = self.arm_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        
        if fraction < 0.9:
            self.get_logger().warn(f"Only {fraction*100}% of path achieved!")
            return False
        
        return self.arm_group.execute(plan, wait=True)

def main():
    rclpy.init()
    node = OpenManipulatorControl()
    
    try:
        # Example 1: Move to home position using joint values
        node.get_logger().info("Moving to home position...")
        joint_goal = [0.0, 0.0, 0.0, 0.0]
        node.move_to_joint_state(joint_goal)
        
        # Example 2: Move to a pose target (IK)
        node.get_logger().info("Moving to pose target...")
        node.set_pose_target(0.3, 0.0, 0.2)
        
        # Example 3: Execute a Cartesian path
        node.get_logger().info("Executing Cartesian path...")
        waypoints = []
        
        wpose = node.arm_group.get_current_pose().pose
        
        # Move forward
        wpose.position.x += 0.1
        waypoints.append(wpose)
        
        # Move down
        wpose.position.z -= 0.1
        waypoints.append(wpose)
        
        # Move back
        wpose.position.x -= 0.1
        waypoints.append(wpose)
        
        node.move_cartesian_path(waypoints)
        
        # Example 4: Gripper control
        node.get_logger().info("Opening gripper...")
        node.move_gripper(0.019)  # Open
        
        node.get_logger().info("Closing gripper...")
        node.move_gripper(0.0)    # Close
        
        # Return to home
        node.get_logger().info("Returning to home position...")
        node.move_to_joint_state([0.0, 0.0, 0.0, 0.0])
        
    except Exception as e:
        node.get_logger().error(f"An error occurred: {str(e)}")
    finally:
        moveit_commander.roscpp_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
