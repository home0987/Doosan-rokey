import os
import sys
import rclpy
import subprocess
import numpy as np
import yaml
from rclpy.node import Node
from rclpy.action import ActionClient
from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import QPixmap, QImage
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped, Pose
from std_msgs.msg import Empty
from nav2_msgs.action import FollowWaypoints
from rescue_control.coordinate import coordinate
from nav2_msgs.srv import SetInitialPose
from PIL import Image, ImageDraw
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class NODE(QThread, Node):
    pose_received = Signal(list)  # 터틀봇 위치 업데이트 신호
    drone_pose_received = Signal(list)  # 드론 위치 업데이트 신호
    log_signal = Signal(str)  # GUI 로그 출력용 시그널

    def __init__(self, node_name='gui_node'):
        super().__init__()
        Node.__init__(self, node_name)

        qos_profile = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.drone_cmd_vel_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)
        self.drone_takeoff_publisher = self.create_publisher(Empty, '/simple_drone/takeoff', 10)
        self.drone_land_publisher = self.create_publisher(Empty, '/simple_drone/land', 10)

        self.waypoint_action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.coord_manager = coordinate()

        # 터틀봇 위치 정보 구독
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, qos_profile)
        
        self.last_amcl_x = None
        self.last_amcl_y = None

        # `/simple_drone/gt_pose` 메시지 타입을 자동 감지
        drone_topic_type = self.detect_drone_topic_type()
        if drone_topic_type == "PoseStamped":
            self.drone_subscription = self.create_subscription(
                PoseStamped, '/simple_drone/gt_pose', self.drone_callback_pose_stamped, qos_profile)
            print("[INFO] /simple_drone/gt_pose 구독: PoseStamped 사용")

        elif drone_topic_type == "Pose":
            self.drone_subscription = self.create_subscription(
                Pose, '/simple_drone/gt_pose', self.drone_callback_pose, qos_profile)
            print("[INFO] /simple_drone/gt_pose 구독: Pose 사용")

        else:
            print("[ERROR] /simple_drone/gt_pose의 메시지 타입을 감지할 수 없음!")

        # AMCL 초기 위치 설정 서비스 클라이언트
        self.set_initial_pose_client = self.create_client(SetInitialPose, "/set_initial_pose")

        # 서비스 활성화 대기
        while not self.set_initial_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /set_initial_pose service...")

        # AMCL 초기 위치 설정 실행
        self.set_initial_pose()

    def detect_drone_topic_type(self):
        try:
            result = subprocess.run(["ros2", "topic", "type", "/simple_drone/gt_pose"],
                                    capture_output=True, text=True, check=True)
            topic_type = result.stdout.strip()
            if "PoseStamped" in topic_type:
                return "PoseStamped"
            elif "Pose" in topic_type:
                return "Pose"
        except:
            return None
        
    def amcl_callback(self, msg):
        """ 터틀봇 위치 정보를 받아 미니맵 갱신 """
        amcl_x = msg.pose.pose.position.x
        amcl_y = msg.pose.pose.position.y

        if self.last_amcl_x is None or abs(self.last_amcl_x - amcl_x) > 0.05 or abs(self.last_amcl_y - amcl_y) > 0.05:
            self.last_amcl_x = amcl_x
            self.last_amcl_y = amcl_y
            self.pose_received.emit([amcl_x, amcl_y])
            print(f"[DEBUG] 터틀봇 위치 수신: x={amcl_x}, y={amcl_y}")  

    def drone_callback_pose_stamped(self, msg):
        """ `/simple_drone/gt_pose`가 PoseStamped 타입일 경우 실행 """
        gt_x = msg.pose.position.x
        gt_y = msg.pose.position.y
        self.process_drone_pose(gt_x, gt_y)

    def drone_callback_pose(self, msg):
        """ `/simple_drone/gt_pose`가 Pose 타입일 경우 실행 """
        gt_x = msg.position.x
        gt_y = msg.position.y
        self.process_drone_pose(gt_x, gt_y)

    def process_drone_pose(self, gt_x, gt_y):
        drone_x = gt_x - (-2.5)
        drone_y = gt_y - 1.5

        print(f"[DEBUG] 드론 위치 수신: x={drone_x}, y={drone_y}")  
        self.drone_pose_received.emit([drone_x, drone_y])  # 시그널 전달

    def set_initial_pose(self):
        """ AMCL 초기 위치 설정 """
        request = SetInitialPose.Request()
        request.pose.header.frame_id = "map"
        request.pose.pose.pose.position.x = 0.0
        request.pose.pose.pose.position.y = 0.0
        request.pose.pose.pose.orientation.w = 1.0

        future = self.set_initial_pose_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            print("[INFO] Initial pose set successfully")
        else:
            print("[WARN] Failed to set initial pose")

    def log(self, message):
        print(message)  # 터미널 출력
        self.log_signal.emit(message)  # GUI로 전달

    def send_waypoint_goal(self):
        waypoints = []
        coordinates = self.coord_manager.get_coordinate()
        for x, y, w in coordinates:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = w
            waypoints.append(pose)
        
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints
        
        self.waypoint_action_client.wait_for_server()
        future = self.waypoint_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.waypoint_goal_callback)
        self.log("[INFO] 순찰 시작")

    def waypoint_goal_callback(self, future):
        goal_handle = future.result()
        if goal_handle and goal_handle.accepted:
            self._goal_handle = goal_handle
            print("[INFO] 웨이포인트 목표가 정상적으로 전송됨.")
        else:
            print("[ERROR] 웨이포인트 목표가 거부됨.")

    def stop_waypoint_goal(self):
        if hasattr(self, "_goal_handle") and self._goal_handle is not None:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done_callback)
            print("[INFO] 순찰 중단 요청을 보냈습니다.")
        else:
            print("[ERROR] 현재 실행 중인 웨이포인트 목표가 없습니다. 강제 정지 실행")
            self.stop_robot()

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if cancel_response and cancel_response.goals_canceling:
            print("[INFO] 순찰 중단 요청이 성공적으로 수행되었습니다.")
            self.stop_robot()
        else:
            print("[ERROR] 순찰 중단 요청이 실패했습니다.")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.log("[INFO] 순찰 정지 명령을 전송했습니다.")

    def start_rescue(self):
        self.drone_takeoff_publisher.publish(Empty())
        self.log("[INFO] 구조 시작 - 드론 이륙")

    def stop_rescue(self):
        self.drone_land_publisher.publish(Empty())
        self.log("[INFO] 구조 중지 - 드론 착륙")

    def run(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.ros_spin)
        self.timer.start(100)  # 100ms마다 실행
        self.exec_() 

    def ros_spin(self):
        rclpy.spin_once(self, timeout_sec=0.1)  # 0.1초마다 ROS 이벤트 실행
        #GUI가 자꾸 강제종료돼서..

class GUI(QMainWindow):
    def __init__(self, ros_thread):
        super().__init__()
        self.ros_thread = ros_thread
        self.ros_thread.log_signal.connect(self.update_log)
        self.ros_thread.pose_received.connect(self.process_signal)  # 터틀봇 시그널 연결
        self.ros_thread.drone_pose_received.connect(self.process_drone_signal) # 드론 시그널 연결
        self.load_map()
        self.setupUi()

    def load_map(self):
        mapping_dir = os.path.join(get_package_share_directory('rescue_control'), 'mapping')
        map_pgm_path = os.path.join(mapping_dir, 'map_wall.pgm')
        map_yaml_path = os.path.join(mapping_dir, 'map_wall.yaml')

        if not os.path.exists(map_pgm_path) or not os.path.exists(map_yaml_path):
            raise FileNotFoundError(f"Map file or YAML not found in {map_pgm_path}")

        image = Image.open(map_pgm_path)
        self.width, self.height = image.size
        self.image_rgb = image.convert('RGB')

        with open(map_yaml_path, 'r') as file:
            data = yaml.safe_load(file)

        self.resolution = data['resolution']
        self.map_x = -data['origin'][0]
        self.map_y = +data['origin'][1] + self.height * self.resolution

        image_gray = image.convert('L')
        image_np = np.array(image_gray)
        image_np[image_np >= 255 * data['occupied_thresh']] = 255
        image_np[image_np <= 255 * data['free_thresh']] = 0
        self.image_rgb = Image.fromarray(np.stack([image_np] * 3, axis=-1), 'RGB')

        self.x = self.map_x
        self.y = self.map_y

    def process_drone_signal(self, message):
        drone_x = message[0]
        drone_y = message[1]

        print(f"[DEBUG] Processing Drone Pose: x={drone_x}, y={drone_y}")

        self.drone_x = self.map_x + (drone_x / self.resolution)
        self.drone_y = self.map_y - (drone_y / self.resolution)

        self.input_image()  # 지도 갱신

    def setupUi(self):
        self.setWindowTitle("Robot Control & Minimap")
        self.setGeometry(100, 100, 800, 400)

        self.centralwidget = QWidget(self)
        self.setCentralWidget(self.centralwidget)

        self.main_layout = QHBoxLayout(self.centralwidget)

        self.textBrowser = QTextBrowser(self.centralwidget)
        self.textBrowser.setFixedSize(250, 250)

        self.minimap_label = QLabel(self.centralwidget)
        self.minimap_label.setFixedSize(250, 250)
        self.minimap_label.setScaledContents(True)

        self.button_layout = QVBoxLayout()

        button_width = 120
        button_height = 40

        self.patrol_button = QPushButton("순찰 시작", self.centralwidget)
        self.patrol_button.setFixedSize(button_width, button_height)
        self.patrol_button.clicked.connect(self.ros_thread.send_waypoint_goal)

        self.patrol_stop_button = QPushButton("순찰 중지", self.centralwidget)
        self.patrol_stop_button.setFixedSize(button_width, button_height)
        self.patrol_stop_button.clicked.connect(self.ros_thread.stop_waypoint_goal)

        self.rescue_button = QPushButton("구조 시작", self.centralwidget)
        self.rescue_button.setFixedSize(button_width, button_height)
        self.rescue_button.clicked.connect(self.ros_thread.start_rescue)

        self.rescue_stop_button = QPushButton("구조 중지", self.centralwidget)
        self.rescue_stop_button.setFixedSize(button_width, button_height)
        self.rescue_stop_button.clicked.connect(self.ros_thread.stop_rescue)

        self.button_layout.addWidget(self.patrol_button)
        self.button_layout.addWidget(self.patrol_stop_button)
        self.button_layout.addWidget(self.rescue_button)
        self.button_layout.addWidget(self.rescue_stop_button)

        self.main_layout.addWidget(self.textBrowser)  # 메시지 로그
        self.main_layout.addWidget(self.minimap_label)  # 미니맵
        self.main_layout.addLayout(self.button_layout)  # 버튼 추가

    def update_log(self, message):
        self.textBrowser.append(message)
        self.textBrowser.ensureCursorVisible()

    def process_signal(self, message):
        """ ROS 2에서 amcl_pose 데이터를 받아 미니맵 업데이트 """
        odom_x = message[0]
        odom_y = message[1]

        print(f"[DEBUG] Processing Pose: x={odom_x}, y={odom_y}")
        self.x = self.map_x + odom_x
        self.y = self.map_y - odom_y

        self.input_image()

    def input_image(self):
        image_copy = self.image_rgb.copy()
        draw = ImageDraw.Draw(image_copy)

        # 터틀봇 위치를 빨간색 점으로 표시
        draw.ellipse((
            self.x / self.resolution - 2,
            self.y / self.resolution - 2,
            self.x / self.resolution + 2,
            self.y / self.resolution + 2),
            fill='red'
        )

        # 드론 위치 파란 점
        if hasattr(self, 'drone_x') and hasattr(self, 'drone_y'):
            draw.ellipse((
                self.drone_x / self.resolution - 2,
                self.drone_y / self.resolution - 2,
                self.drone_x / self.resolution + 2,
                self.drone_y / self.resolution + 2),
                fill='blue'
            )
        
        image_resized = image_copy.resize((250, 250))
        pil_image = image_resized.convert('RGBA')
        data = pil_image.tobytes("raw", "RGBA")
        qimage = QImage(data, 250, 250, QImage.Format_RGBA8888)
        pixmap = QPixmap.fromImage(qimage)

        self.minimap_label.setPixmap(pixmap)
        self.minimap_label.update()
        self.minimap_label.repaint()


def main():
    rclpy.init()
    node = NODE()
    node.start()

    app = QApplication(sys.argv)
    gui = GUI(node)
    gui.show()

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
