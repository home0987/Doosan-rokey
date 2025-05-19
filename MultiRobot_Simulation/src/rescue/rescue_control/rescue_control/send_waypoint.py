import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped, Point
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import Bool
import math
import threading
import sys
import select
import termios
import tty
#from coordinate import coordinate
from rescue_control.coordinate import coordinate

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Point, '/current_pose', 10)
        self.bool_subscription = self.create_subscription(Bool, '/detected', self.detect_callback, 10)
        self.init_pose = [0.0, 0.0, 0.0, 1.0]
        self.coord_manager = coordinate()
        self.subscription
        self.bool_subscription
        self._goal_handle = None
        self.detect = False
        self.x = 0.0
        self.y = 0.0

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.get_logger().info(f"Updated Pose: x={self.x}, y={self.y}")

    def detect_callback(self, msg):
        self.detect = msg.data

        if self.detect == True:
            self.cancel_goal()
    def send_goal(self):
        waypoints = []
        coordinates = self.coord_manager.get_coordinate()

        for coord in coordinates:
            x, y, w = coord  # yaw 값을 추가적으로 받는다고 가정 x, y, yaw = coord
            yaw = 0.0
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = w
            # pose.pose.orientation = self.euler_to_quaternion(0, 0, yaw)
            waypoints.append(pose)

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.action_client.wait_for_server()
        
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted.')
        self._goal_handle = goal_handle

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Waypoint Index: {feedback.current_waypoint}')
        
    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Attempting to cancel the goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal to cancel.')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal cancellation accepted. Exiting program...')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)  # Exit the program after successful cancellation
        else:
            self.get_logger().info('Goal cancellation failed or no active goal to cancel.')   

    def get_result_callback(self, future):
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        if missed_waypoints:
            self.get_logger().info(f'Missed waypoints: {missed_waypoints}')
        else:
            self.get_logger().info('All waypoints completed successfully!')
            self.publish_callback()

    def publish_callback(self):
        point_msg = Point()
        point_msg.x = self.x 
        point_msg.y = self.y
        point_msg.z = 0.0
        self.get_logger().info(f"Publishing Pose: x={point_msg.x}, y={point_msg.y}, z={point_msg.z}")
        self.publisher.publish(point_msg)

def keyboard_listener(node):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'g':
                    node.get_logger().info('Key "g" pressed. Sending goal...')
                    node.send_goal()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

