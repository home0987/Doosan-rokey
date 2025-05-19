import sys
import rclpy
import threading
import math
from std_msgs.msg import Bool

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.srv import SetInitialPose
from geometry_msgs.msg import Point, Quaternion

from PySide2.QtWidgets import QApplication, QMainWindow, QPushButton, QTextBrowser, QWidget, QVBoxLayout


class Waypoint(Node):
    def __init__(self):
        super().__init__('waypoint')
        
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self._goal_handle = None  # 목표 핸들 저장
        self.init_pose = [0.5900933441049189, 0.5607035334341172, -0.5567676994049192, 0.8306682423804064]
        # self.wait_for_map()

        self.set_initial_pose_service_client = self.create_client(
            SetInitialPose, 
            '/set_initial_pose'
            )
        while not self.set_initial_pose_service_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info('Service /set_initial_pose not available, waiting again...')
        self.set_initial_pose(*self.init_pose)

        self.publisher_ = self.create_publisher(Bool, '/bool_topic', 10)
        # self.timer = self.create_timer(1.0, self.start_callback)
        self.state = True  # 초기 상태

    
    def start_callback(self):
        msg = Bool()
        self.state = True
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

    def stop_callback(self):
        msg = Bool()
        self.state = False
        msg.data = self.state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')


    def set_initial_pose(self, x, y, z, w):
        req = SetInitialPose.Request()
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)

        # Covariance 설정 유지
        req.pose.pose.covariance = [
            0.1827164537828576, -0.005612746039706129, 0.0, 0.0, 0.0, 0.0,
            -0.005612746039706129, 0.18435926476307263, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.06971217188152826, 0.0
        ]

        future = self.set_initial_pose_service_client.call_async(req)
        
        # 서비스 응답을 기다림
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info("[INFO] Initial pose set successfully")
        else:
            self.get_logger().warn("[WARN] Failed to set initial pose")

    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """ Euler 각도를 쿼터니언으로 변환 """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def send_goal(self):
        """ 목표 지점으로 이동 시작 """
        waypoints = []

        # 3개의 웨이포인트 정의
        positions = [
            (-0.366, -0.189, 0.00599878557282514, 0.9999820071239538),
            (0.404, -0.117, 0.026381041713097044, 0.9996519597530592),
            (0.5900933441049189, 0.5607035334341172, -0.5567676994049192, 0.8306682423804064)

        ]

        for pos in positions:
            waypoint = PoseStamped()
            waypoint.header.frame_id = "map"
            waypoint.pose.position.x = pos[0]
            waypoint.pose.position.y = pos[1]
            waypoint.pose.orientation.z = pos[2]
            waypoint.pose.orientation.w = pos[3]
            waypoints.append(waypoint)

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """ 목표 지점 수락 여부 확인 """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
        else:
            self.get_logger().info('Goal accepted :)')
            self._get_result_future = self._goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """ 목표 지점 도달 상황 피드백 """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Waypoint Index: {feedback.current_waypoint}')

    def cancel_goal(self):
        """ 목표 지점 이동 취소 """
        if self._goal_handle is not None:
            self.get_logger().info('Cancelling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal to cancel.')

    def cancel_done_callback(self, future):
        """ 목표 취소 완료 확인 """
        cancel_response = future.result()
        if len(cancel_response.goals_cancelled) > 0:
            self.get_logger().info('Goal cancellation accepted.')
        else:
            self.get_logger().info('Goal cancellation failed or no active goal to cancel.')

    def get_result_callback(self, future):
        """ 목표 완료 결과 확인 """
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        if missed_waypoints:
            self.get_logger().info(f'Missed waypoints: {missed_waypoints}')
        else:
            self.get_logger().info('All waypoints completed successfully!')


class WaypointGUI(QMainWindow):
    def __init__(self, node):
        super().__init__()

        self.node = node
        self.setWindowTitle("Waypoint Navigation")
        self.setGeometry(150, 150, 800, 600)

        # 중앙 위젯 설정
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout(central_widget)

        # 시작 버튼
        self.cancel_button = QPushButton("시작", self)
        self.cancel_button.setFixedSize(800, 100)  # 가로 200px, 세로 60px
        self.cancel_button.clicked.connect(self.start_navigation)
        self.cancel_button.setStyleSheet("font-size: 24px; font-weight: bold;")

        layout.addWidget(self.cancel_button)

        # 정지 버튼
        self.cancel_button = QPushButton("정지", self)
        self.cancel_button.setFixedSize(800, 100)  # 가로 200px, 세로 60px
        self.cancel_button.clicked.connect(self.stop_navigation)
        self.cancel_button.setStyleSheet("font-size: 24px; font-weight: bold;")

        layout.addWidget(self.cancel_button)

        # 반납 버튼
        self.start_button = QPushButton("반납", self)
        self.start_button.setFixedSize(800, 100)  # 가로 200px, 세로 60px
        self.start_button.clicked.connect(self.return_navigation)
        self.start_button.setStyleSheet("font-size: 24px; font-weight: bold;")

        layout.addWidget(self.start_button)

        # 상태 메시지 출력
        # self.text_browser = QTextBrowser(self)
        # layout.addWidget(self.text_browser)

    def return_navigation(self):
        """ 'return_navigation ' 버튼 클릭 시 실행 """
        self.node.get_logger().info("return_navigation ...")
        self.node.send_goal()

    def start_navigation(self):
        """ 'start_navigation ' 버튼 클릭 시 실행 """
        self.node.get_logger().info("start_navigation ...")
        self.node.start_callback()
    
    def stop_navigation(self):
        """ 'stop_navigation ' 버튼 클릭 시 실행 """
        self.node.get_logger().info("stop_navigation ...")
        self.node.stop_callback()



def main(args=None):
    print("waypoint start1")
    rclpy.init(args=args)
    node = Waypoint()

    # ROS 2 스레드 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # GUI 실행
    app = QApplication(sys.argv)
    gui = WaypointGUI(node)
    gui.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

