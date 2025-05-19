import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge
import time

class RedObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('red_object_tracking')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # 이동 명령 퍼블리셔 생성
        self.subscription = self.create_subscription(
            Image, '/camera_sensor/image_raw', self.image_callback, 10)  # 카메라 토픽 구독
        self.bool_publisher_ = self.create_publisher(Bool, '/detected', 10)

        self.bridge = CvBridge()  # ROS 이미지를 OpenCV 형식으로 변환
        self.target_distance = 0.15  # 유지할 목표 거리 (cm)
        self.frame_width = 640  # 프레임 너비 설정
        self.movement_step = 10  # 10cm씩 이동 (Gazebo 시뮬레이션용)
        self.is_moving = False  # 이동 여부를 나타내는 플래그

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
            
            self.get_logger().info(f"거리: {distance:.2f} cm")
            # 바운딩 박스 그리기
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # 주요 색상 추출
            dominant_color = self.get_dominant_color(frame[y:y+h, x:x+w])
            self.get_logger().info(f"주요 색상: {dominant_color}")
            self.detect_obj()
            
            # 10cm씩 이동 후 정지
            if not self.is_moving:
                move_cmd = self.compute_movement(x_center, distance)
                self.publisher_.publish(move_cmd)
                self.is_moving = True
                time.sleep(1.0)
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
        
        move_cmd.linear.x = 0.3  # 10cm 전진
        move_cmd.angular.z = 0.0
        
        center_threshold = self.frame_width * 0.2  # 중심 오차 범위 설정
        if x_center < self.frame_width / 2 - center_threshold:
            move_cmd.angular.z = 0.2  # 왼쪽 회전
        elif x_center > self.frame_width / 2 + center_threshold:
            move_cmd.angular.z = -0.2  # 오른쪽 회전
        else:
            move_cmd.angular.z = 0.0  # 직진 유지
        move_cmd = Twist() if distance<200.00 else move_cmd  #150이 타일 한칸정도 
        return move_cmd
    
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

def main(args=None):
    rclpy.init(args=args)
    node = RedObjectTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
