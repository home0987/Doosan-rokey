import os
import sys
import rclpy
import subprocess
import numpy as np
import yaml
from rclpy.node import Node
from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import QPixmap, QImage
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped, Pose
from nav2_msgs.srv import SetInitialPose
from PIL import Image, ImageDraw


class NODE(QThread, Node):
    pose_received = Signal(list)  # 터틀봇 위치 업데이트 신호
    drone_pose_received = Signal(list)  # 드론 위치 업데이트 신호

    def __init__(self, node_name='gui_node'):
        super().__init__()
        Node.__init__(self, node_name)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)  # 터틀봇 이동
        self.drone_cmd_vel_publisher = self.create_publisher(Twist, '/simple_drone/cmd_vel', 10)  # 드론 이동

        # 터틀봇 위치 정보 구독
        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, 10)

        # `/simple_drone/gt_pose` 메시지 타입을 자동 감지
        drone_topic_type = self.detect_drone_topic_type()
        if drone_topic_type == "PoseStamped":
            self.drone_subscription = self.create_subscription(
                PoseStamped, '/simple_drone/gt_pose', self.drone_callback_pose_stamped, 10)
            print("[INFO] /simple_drone/gt_pose 구독: PoseStamped 사용")

        elif drone_topic_type == "Pose":
            self.drone_subscription = self.create_subscription(
                Pose, '/simple_drone/gt_pose', self.drone_callback_pose, 10)
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
        """ `/simple_drone/gt_pose`의 메시지 타입을 자동 탐지하여 반환 """
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
        print(f"[DEBUG] 터틀봇 위치 수신: x={amcl_x}, y={amcl_y}")  
        self.pose_received.emit([amcl_x, amcl_y])  # 시그널 전달

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

    def send_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        """ 터틀봇 이동 명령 """
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_publisher.publish(twist)
        print(f"[INFO] /cmd_vel → linear_x={linear_x}, angular_z={angular_z}")

    def send_drone_cmd_vel(self, linear_x=0.0, linear_z=0.0):
        """ 드론 이동 명령 """
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.z = linear_z
        self.drone_cmd_vel_publisher.publish(twist)
        print(f"[INFO] /simple_drone/cmd_vel → linear_x={linear_x}, linear_z={linear_z}")

    def run(self):
        """ ROS 2 노드를 별도 스레드에서 실행 """
        rclpy.spin(self)

class GUI(QMainWindow):
    def __init__(self, ros_thread):
        super().__init__()
        self.ros_thread = ros_thread
        self.ros_thread.pose_received.connect(self.process_signal)  # 터틀봇 시그널 연결
        self.ros_thread.drone_pose_received.connect(self.process_drone_signal) # 드론 시그널 연결
        self.load_map()
        self.setupUi()

    def load_map(self):
        """ 미니맵용 지도 로드 """
        map_pgm_path = "/home/rokey/mapping/new_yjz_map.pgm"
        map_yaml_path = "/home/rokey/mapping/new_yjz_map.yaml"

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
        """ 드론의 위치를 받아 미니맵 업데이트 """
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

        # 메인 레이아웃
        self.main_layout = QHBoxLayout(self.centralwidget)

        # 텍스트 브라우저 (메시지 로그)
        self.textBrowser = QTextBrowser(self.centralwidget)
        self.textBrowser.setFixedSize(250, 250)

        # 미니맵 QLabel 추가
        self.minimap_label = QLabel(self.centralwidget)
        self.minimap_label.setFixedSize(250, 250)
        self.minimap_label.setScaledContents(True)  # 이미지 크기 조정 가능하도록 설정

        self.button_layout = QVBoxLayout()

        button_width = 120
        button_height = 40

        self.patrol_button = QPushButton("순찰 시작", self.centralwidget)
        self.patrol_button.setFixedSize(button_width, button_height)
        self.patrol_button.clicked.connect(lambda: print("순찰 시작"))

        self.patrol_stop_button = QPushButton("순찰 중지", self.centralwidget)
        self.patrol_stop_button.setFixedSize(button_width, button_height)
        self.patrol_stop_button.clicked.connect(lambda: print("순찰 중지"))

        self.rescue_button = QPushButton("구조 시작", self.centralwidget)
        self.rescue_button.setFixedSize(button_width, button_height)
        self.rescue_button.clicked.connect(lambda: print("구조 시작"))

        self.rescue_stop_button = QPushButton("구조 중지", self.centralwidget)
        self.rescue_stop_button.setFixedSize(button_width, button_height)
        self.rescue_stop_button.clicked.connect(lambda: print("구조 중지"))

        self.button_layout.addWidget(self.patrol_button)
        self.button_layout.addWidget(self.patrol_stop_button)
        self.button_layout.addWidget(self.rescue_button)
        self.button_layout.addWidget(self.rescue_stop_button)

        self.main_layout.addWidget(self.textBrowser)  # 메시지 로그
        self.main_layout.addWidget(self.minimap_label)  # 미니맵
        self.main_layout.addLayout(self.button_layout)  # 버튼 추가


    def process_signal(self, message):
        """ ROS 2에서 amcl_pose 데이터를 받아 미니맵 업데이트 """
        odom_x = message[0]
        odom_y = message[1]

        print(f"[DEBUG] Processing Pose: x={odom_x}, y={odom_y}")
        self.x = self.map_x + odom_x
        self.y = self.map_y - odom_y

        self.input_image()

    def input_image(self):
        """ 미니맵을 업데이트하고 QLabel에 표시 """
        image_copy = self.image_rgb.copy()
        draw = ImageDraw.Draw(image_copy)

        # 현재 위치를 빨간색 점으로 표시
        draw.ellipse((
            self.x / self.resolution - 2,
            self.y / self.resolution - 2,
            self.x / self.resolution + 2,
            self.y / self.resolution + 2),
            fill='red'
        )

        if hasattr(self, 'drone_x') and hasattr(self, 'drone_y'):
            draw.ellipse((
                self.drone_x / self.resolution - 2,
                self.drone_y / self.resolution - 2,
                self.drone_x / self.resolution + 2,
                self.drone_y / self.resolution + 2),
                fill='blue'
            )
        

        # QPixmap 변환 및 QLabel 업데이트
        image_resized = image_copy.resize((250, 250))
        pil_image = image_resized.convert('RGBA')
        data = pil_image.tobytes("raw", "RGBA")
        qimage = QImage(data, 250, 250, QImage.Format_RGBA8888)
        pixmap = QPixmap.fromImage(qimage)

        self.minimap_label.setPixmap(pixmap)
        self.minimap_label.update()
        self.minimap_label.repaint()


    def start_patrol(self):
        """ 순찰 시작 - 터틀봇 전진 """
        print("[INFO] 순찰 시작 - 터틀봇 이동")
        self.ros_thread.send_cmd_vel(linear_x=0.2)  # 전진 속도 0.2m/s

    def stop_patrol(self):
        """ 순찰 중지 - 터틀봇 정지 """
        print("[INFO] 순찰 중지 - 터틀봇 정지")
        self.ros_thread.send_cmd_vel(linear_x=0.0)  # 정지

    def start_rescue(self):
        """ 구조 시작 - 드론 상승 """
        print("[INFO] 구조 시작 - 드론 이동")
        self.ros_thread.send_drone_cmd_vel(linear_x=0.5)

    def stop_rescue(self):
        """ 구조 중지 - 드론 정지 """
        print("[INFO] 구조 중지 - 드론 정지")
        self.ros_thread.send_drone_cmd_vel(linear_x=0.0)  # 정지


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
