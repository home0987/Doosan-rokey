import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge
import time, math, threading

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class RedObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('red_object_tracking')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # 이동 명령 퍼블리셔 생성
        self.subscription = self.create_subscription(
            Image, '/camera_sensor/image_raw', self.image_callback, 10)  # 카메라 토픽 구독
        self.bool_publisher_ = self.create_publisher(Bool, '/detected', 10)
        # self.sub_amcl_pose = self.create_subscription(
        #     PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl_pose, 10)

        self.bridge = CvBridge()  # ROS 이미지를 OpenCV 형식으로 변환
        self.target_distance = 0.15  # 유지할 목표 거리 (cm)
        self.frame_width = 480  # 프레임 너비 설정
        self.movement_step = 10  # 10cm씩 이동 (Gazebo 시뮬레이션용)
        self.is_moving = False  # 이동 여부를 나타내는 플래그
        self.nav_mode = False   # send goal 실행여부
        self.amcl_current_pose = PoseWithCovarianceStamped().pose.pose

    def amcl_pose_listener(self):
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.cb_amcl_pose, 10)
        

    def cb_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """AMCL 위치 갱신"""
        self.amcl_current_pose = msg.pose.pose
        self.get_logger().info(f"amcl pos x = {self.amcl_current_pose.position.x}, y = {self.amcl_current_pose.position.y}, z = {self.amcl_current_pose.position.z}")

    def image_callback(self, msg):
        """ 카메라 토픽에서 이미지를 받아와 처리 """
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.resize(frame, (640, 640))  # 프레임 크기 조정
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # HSV 색공간 변환
        
        # 빨간색 HSV 범위 설정 (두 개의 범위 사용)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)  # 빨간색 마스크 생성 (첫 번째 범위)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)  # 빨간색 마스크 생성 (두 번째 범위)
        red_mask = mask1 + mask2  # 두 개의 마스크 합치기
        
        # 컨투어(객체의 외곽선) 찾기
        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)  # 가장 큰 객체 선택
            x, y, w, h = cv2.boundingRect(largest_contour)  # 바운딩 박스 생성
            x_center = x + w // 2  # 객체 중심 좌표 계산
            distance = self.estimate_distance(h)  # 객체와의 거리 계산
            
            #self.get_logger().info(f"거리: {distance:.2f} cm")
            # 바운딩 박스 그리기
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # 주요 색상 추출
            dominant_color = self.get_dominant_color(frame[y:y+h, x:x+w])
            #self.get_logger().info(f"주요 색상: {dominant_color}")
            self.detect_obj()
            
            # 10cm씩 이동 후 정지
            if not self.is_moving and not self.nav_mode:
                move_cmd = self.compute_movement(x_center, distance)
                if move_cmd is not None : 
                    self.publisher_.publish(move_cmd)
                    self.is_moving = True
                    time.sleep(0.5)
                    self.stop_after_movement()  # 0.1초 정지
               
            else:
                # self.stop_drone()
                pass
        
        cv2.imshow("Red Object Tracking", frame)
        cv2.waitKey(1)

    def estimate_distance(self, bbox_height):
        """ 객체와의 거리를 추정 """
        focal_length = 554  # 초점 거리 (조정 필요)
        known_height = 44  # cm (실제 물체의 높이) ### 수정
        return (known_height * focal_length) / bbox_height if bbox_height > 0 else 100

    def get_dominant_color(self, roi):
        """ ROI(관심영역)에서 주요 색상을 추출 """
        if roi is None or roi.size == 0:
            return (0, 0, 0)
        pixels = np.float32(roi.reshape(-1, 3))
        n_colors = 1
        _, labels, palette = cv2.kmeans(pixels, n_colors, None, 
                                         (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0), 
                                         10, cv2.KMEANS_RANDOM_CENTERS)
        return tuple(map(int, palette[0]))

    def compute_movement(self, x_center, distance):
        """ 객체를 따라가도록 이동 명령을 계산 """
        move_cmd = Twist()
        
        move_cmd.linear.x = 0.0  # 10cm 전진
        move_cmd.angular.z = 0.0
        
        center_threshold = self.frame_width * 0.2  # 중심 오차 범위 설정
        if x_center < self.frame_width / 2 - center_threshold:
            move_cmd.angular.z = 0.2  # 왼쪽 회전
            self.get_logger().info(f"angular z = {move_cmd.angular.z}")
            return move_cmd
        elif x_center > self.frame_width / 2 + center_threshold:
            move_cmd.angular.z = -0.2  # 오른쪽 회전
            self.get_logger().info(f"angular z = {move_cmd.angular.z}")
            return move_cmd
        else:
            if distance > 200.0 :
                self.nav_mode = True
                self.send_goal()
                return
            else : 
                return Twist()
    
    def stop_after_movement(self):
        """ 10cm 이동 후 정지하고 다시 객체 탐색 """
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        self.publisher_.publish(move_cmd)
        self.is_moving = False
        self.get_logger().info("10cm 이동 후 다시 객체 탐색")

    def detect_obj(self):
        send_detect = Bool()
        self.detected = True
        send_detect.data = self.detected
        self.bool_publisher_.publish(send_detect)  
    # def stop_robot(self):
    #     """ 객체를 찾지 못했을 때 정지 """
    #     move_cmd = Twist()
    #     move_cmd.linear.x = 0.0
    #     move_cmd.angular.z = 0.0
    #     self.publisher_.publish(move_cmd)
    #     self.get_logger().info("빨간색을 찾지 못해 정지")

    ###send goal##
    def send_goal(self):
        time.sleep(0.5)
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        quat = self.amcl_current_pose.orientation
        _, _, yaw_rad = self.euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)

        goal_msg.pose.pose.position.x = self.amcl_current_pose.position.x + 1.5 * math.cos(yaw_rad)
        goal_msg.pose.pose.position.y = self.amcl_current_pose.position.y + 1.5 * math.sin(yaw_rad)

        goal_msg.pose.pose.orientation.z = self.amcl_current_pose.orientation.z
        goal_msg.pose.pose.orientation.w = self.amcl_current_pose.orientation.w

        self.get_logger().info(f"yaw = {yaw_rad}rad, {math.degrees(yaw_rad)}deg")
        self.get_logger().info(f"current pos x = {self.amcl_current_pose.position.x}, y = {self.amcl_current_pose.position.y}")
        self.get_logger().info(f"Navigating to point x = {goal_msg.pose.pose.position.x}, y = {goal_msg.pose.pose.position.y}, yaw_rad = {yaw_rad}")

        # 서버 대기 및 목표 전송
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, self.goal_feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_feedback_callback(self, feedback_msg): pass
  
    def goal_response_callback(self, future):
        """ 목표가 수락되었는지 확인 """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected.")
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        self._goal_handle = goal_handle  # 현재 실행 중인 goal 핸들 저장
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """ 목표 도착 """
        self.nav_mode = False
        self.get_logger().info(f"reached point x = {self.amcl_current_pose.position.x}, y = {self.amcl_current_pose.position.y}")

    def euler_from_quaternion(self, x, y, z, w):
        """
        쿼터니언(x, y, z, w)을 오일러 각도(roll, pitch, yaw)로 변환하는 함수
        """
        # Roll (x축 회전)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y축 회전)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # 범위를 [-π/2, π/2]로 제한
        else:
            pitch = math.asin(sinp)

        # Yaw (z축 회전)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    node = RedObjectTrackingNode()
    amcl_thread = threading.Thread(target=node.amcl_pose_listener, daemon=True)
    amcl_thread.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    # rclpy.init(args=args)
    # executor = MultiThreadedExecutor()
    # node = RedObjectTrackingNode()
    # executor.add_node(node)
    # try:
    #     executor.spin()
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

if __name__ == '__main__':
    main()
