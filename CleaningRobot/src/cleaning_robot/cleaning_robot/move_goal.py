import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class MoveGoal(Node):
    def __init__(self):
        super().__init__('move_goal')

        # PoseWithCovarianceStamped 메시지를 구독
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/nearest_unknown', 
            self.pose_callback, 
            10
        )

        # Nav2의 goal_pose 퍼블리셔
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Navigation2 Action Client
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def pose_callback(self, msg):
        """
        PoseWithCovarianceStamped 메시지를 PoseStamped로 변환 후 Nav2 목표로 설정
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.get_logger().info(f"Received pose: x={x}, y={y}")

        # 목표 메시지 생성
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"  # 전역 좌표계 사용

        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation = msg.pose.pose.orientation  # 기존 회전 값 사용

        # goal_pose 퍼블리시
        self.goal_publisher.publish(goal_msg)

        # Nav2 Action을 통한 목표 전송
        self.send_goal(goal_msg)

    def send_goal(self, goal_pose):
        """
        Navigation2에 목표 좌표를 Action Client로 전송
        """
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Navigation2 action server not available!")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f"Sending goal: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")
        send_goal_future = self.action_client.send_goal_async(goal_msg)

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        목표 전송 후 응답 처리
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the server.")
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """
        목표 도달 결과 처리
        """
        result = future.result().result
        self.get_logger().info(f"Goal result: {result}")

def main(args=None):
    rclpy.init(args=args)
    node = MoveGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
