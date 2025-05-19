import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import GripperCommand
import time

class GripperActionClient(Node):
    def __init__(self):
        super().__init__('gripper_action_client')
        
        # Create action client for gripper controller
        self._action_client = ActionClient(
            self, 
            GripperCommand, 
            '/gripper_controller/gripper_cmd'
        )

    def gripper_open(self,position=0.019, max_effort=10.0):
        self.send_goal(position, max_effort)

    def gripper_close(self,position=-0.01, max_effort=10.0):
        self.send_goal(position, max_effort)

    def send_goal(self, position=-0.01, max_effort=10.0):
        # Wait for action server to be available
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return None

        # Create goal message
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        # Send goal and wait for result
        self.get_logger().info(f'Sending goal: position={position}, max_effort={max_effort}')
        future = self._action_client.send_goal_async(goal_msg)
        
        # Block until goal is sent
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return None

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        return result

def main(args=None):
    rclpy.init(args=args)
    
    gripper_client = GripperActionClient()
    
    try:
        while True:
            user_input = input("Enter enter to continue or 'quit' to exit: ")
            if user_input == 'quit':
                break

            # Send goal with default parameters
            gripper_client.gripper_open()
            time.sleep(10)
            gripper_client.gripper_close()

    except Exception as e:
        gripper_client.get_logger().error(f'Error: {str(e)}')
    finally:
        # Cleanup
        gripper_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
