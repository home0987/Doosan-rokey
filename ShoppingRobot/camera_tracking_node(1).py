# camera_tracking_node.py

# amr -> amclpose topic을 주차장에서 날리기,,

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # Odometry 메시지 추가
# Odometry는 로봇의 위치와 자세(orientation)를 추정하는 기술
#  로봇이 시작한 위치에서 이동한 거리를 추정
import cv2
import numpy as np
from ultralytics import YOLO
from geometry_msgs.msg import Point

from geometry_msgs.msg import Quaternion

class CameraTrackingNode(Node):
    def __init__(self):
        super().__init__('camera_tracking_node')
        
        # <1.퍼블리시 Twist /cmd_vel>
        # 퍼블리셔 생성: 이동 명령 및 위치 정보 퍼블리시
        self.publisher_ = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)  
        
        # <2. 퍼블리시 Odometry /odometry>
        # 이동 명령 퍼블리셔
        self.odom_publisher_ = self.create_publisher(
            Odometry,
            '/odometry',
            10)  # 위치 정보 퍼블리셔
        
        # 이동 명령(Twist)과 위치 정보(Odometry)를 퍼블리시하는 역할
        self.timer = self.create_timer(0.1, self.process_frame)  # 0.1초마다 실행
        # 카메라 열기
        self.cap = cv2.VideoCapture(4)  

        # 카메라 열기 실패 처리
        if not self.cap.isOpened():
            self.get_logger().error("카메라 열기 실패!")
            return
        
        self.model = YOLO('/home/kante/shopping_pat_ws/runs/detect/amr_train/weights/best.pt')

        
        # 카메라 캘리브레이션 필요

        # focal_length <- focal_length_measurement.py 로 구함 / 픽셀 단위임.
        self.focal_length = 250 # 초점 거리 (실제 측정 필요) # <== 이거 알아내야 함.
        # / 임의로 수정한 상태, -> 이 값일때가 실제 거리에 가까움,, 
        self.known_height = 10  # AMR 실제 높이 (cm)
        self.target_distance = 15  # 유지할 목표 거리 (cm)

        # 로봇의 초기 위치 (예시)
        self.robot_x = 0.0  # X 위치 (m)
        self.robot_y = 0.0  # Y 위치 (m)
        self.robot_theta = 0.0  # 로봇의 방향 (회전각, rad)

    # 객체 색상 정보 함수
    def get_dominant_color(self, image, k=3):
        """ 주어진 이미지에서 주요 색상을 찾는 함수 """
        if image is None or image.size == 0:
            return (255, 255, 255)  # 분석 불가한 경우 흰색 반환

        pixels = np.float32(image.reshape(-1, 3))
        if len(pixels) < k:
            return tuple(map(int, pixels.mean(axis=0)))

        _, labels, palette = cv2.kmeans(
            pixels, k, None,
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0), 
            10, cv2.KMEANS_RANDOM_CENTERS
        )
        
        _, counts = np.unique(labels, return_counts=True)
        dominant_color = palette[np.argmax(counts)]
        return tuple(map(int, dominant_color))

    # 특정 객체 + 목표 색상 -> 목표색에 두드러지면 TRUE
    def is_target_color(self, color, target='red'):
        """ 특정 색상이 목표 색상인지 판단하는 함수 """
        b, g, r = color  # OpenCV는 BGR 형식

        if target == 'red':
            return r > 100 and r > g and r > b  # 빨간색이 두드러지면 True
        elif target == 'blue':
            return b > 100 and b > g and b > r
        elif target == 'green':
            return g > 100 and g > r and g > b
        return False

    # 토픽 관련 // 
    # 퍼블리시 _ 콜백함수,,
    '''
    학습한 모델로 객체를 추적 -> 객체 색상 분석 + 라벨이 0
    => 객체 중심 좌표 계산 / 거리 계산 (삼각측량 공식)

    '''
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # YOLO 입력 크기에 맞게 리사이즈
        frame = cv2.resize(frame, (640, 640))
        self.frame_width = frame.shape[1]

        # YOLO 객체 추적 수행
        results = self.model.track(frame, persist=True)
        labels = results[0].boxes.cls  # 첫 번째 객체의 클래스 번호
        confidences = results[0].boxes.conf  # 신뢰도 값
        class_names = self.model.names  # 모델에서 학습된 클래스명들

        '''
        문제 : 현재 객체 탐지가 너무 많이 되어 과부화가 올수 있는 문제가 있음,,
        '''
        # 1> 감지된 객체가 있는 경우
        if results and len(results) > 0 and hasattr(results[0], 'boxes'):
            boxes = results[0].boxes

            for box, label in zip(boxes, labels):
                if int(label) == 0:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    print(f"바운딩 박스 좌표: ({x1}, {y1}), ({x2}, {y2}) | 클래스: {label}")
                    bbox_height = y2 - y1  # 바운딩 박스 높이 (픽셀)

                ## ROS2 Error: UnboundLocalError: local variable 'y1' referenced before assignment'
                # 여러 바운딩 박스에 대해서, no objects에 대한 바운딩 박스? 정보도 같이 들어와서 문제가 생기는 듯
                # -> 그래서 y1 생성전에 if int(label) == 0, 줬는데, 
                #    dominant_color = self.get_dominant_color(frame[y1:y2, x1:x2])
                #UnboundLocalError: local variable 'y1' referenced before assignment

                    # 1-1>색상 분석
                    dominant_color = self.get_dominant_color(frame[y1:y2, x1:x2])
                    # dominant_color==RED + 라벨이0_사람이면,
                    # 추가: target <-여기에 사용자 색상 선택을 줌,
                    if self.is_target_color(dominant_color, target='red') and int(label) == 0:
                        #
                        # 객체 중심 좌표 계산
                        x_center, y_center = (x1 + x2) // 2, (y1 + y2) // 2
                        cv2.circle(frame, (x_center, y_center), 5, (0,255,0), -1)
                        
                        # 거리 계산 (삼각측량 공식)
                        # ==> 바운딩 박스 길이가 고려된 계산법,
                        '''
                        < 아이디어 . 1>
                        예를 들어 바운딩 박스 높이가 100픽셀이고,
                        focal_length = 75
                        known_height = 5인 경우,
                        
                        distance = 5 * 75 / 100 = 3.75

                        해당 공식은, 객체 이동 + 카메라 이동으로
                        바운딩 박스 높이가 게속 달라져서 바운딩 박스 높이가 변하는?문제가 있을수도
                        '''
                        if bbox_height > 0:
                            # distance 생성 부분,,
                            distance = (self.known_height * self.focal_length) / bbox_height
                            self.get_logger().info(f"거리: {distance:.2f} cm")

                            # 바운딩 박스 중심 계산
                            # 바운딩 박스가 치우치면, 움직임을 보정하여 토픽으로 보내기 위함.
                            x_center = (x1 + x2) / 2
                            move_cmd = self.compute_movement(x_center, distance)

                            ## <1> /cmd_vel
                            # ROS2 이동 명령 퍼블리시
                            self.publisher_.publish(move_cmd)
                            self.get_logger().info(
                                f"여기는 /odometry topic publish 입니다, {move_cmd} "
                            )

                            ## <2> /odometry
                            # ROS2위치 정보 퍼블리시
                            odom_msg = Odometry()
                            odom_msg.header.stamp = self.get_clock().now().to_msg()
                            odom_msg.header.frame_id = "odom"

                            # 위치 정보 예제 (기본 값으로 설정)
                            odom_msg.pose.pose.position.x = distance
                            odom_msg.pose.pose.position.y = 0.0
                            odom_msg.pose.pose.position.z = 0.0

                            self.odom_publisher_.publish(odom_msg)
                            self.get_logger().info(f"/odometry 발행: x={odom_msg.pose.pose.position.x}")

                            # 바운딩 박스 및 거리 표시
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            label_text = f"Distance: {distance:.2f} cm"
                            # label_text_bbox = f"BBOX height: {bbox_height:.2f} px" # test,,
                            cv2.putText(frame, label_text, (x1, y1 - 10), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            # cv2.putText(frame, label_text_bbox, (x1, y1 - 10), 
                            #             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    else:
                        # **빨간색이 아닐 경우 이동 X**
                        self.get_logger().info("빨간색 객체가 아님. 이동 명령 X.")



        # 2> 객체 인지 못함. 
        # 현재 위치를 /twist 토픽으로 보내야 하므로,
        else:
            self.get_logger().info('객체 못찾음, 정지')
            # 객체를 못 찾았을 때 기본 명령 (정지 명령)과 위치 정보 퍼블리시
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.publisher_.publish(move_cmd)
            self.get_logger().info("객체를 못 찾았음. 정지.")
            
            # 현재 위치를 Odometry 메시지로 퍼블리시
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            
            # 로봇의 위치와 방향 설정
            odom_msg.pose.pose.position = Point(x=self.robot_x, y=self.robot_y, z=0.0)
            odom_msg.pose.pose.orientation = self.get_quaternion_from_euler(0.0, 0.0, self.robot_theta)

            # Odometry 메시지 퍼블리시
            self.odom_publisher_.publish(odom_msg)

        # 화면 출력
        cv2.imshow("Tracking", frame)
        cv2.waitKey(1)

    # 오일러 -> 쿼터니언
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        # Roll, pitch, yaw (Euler Angles)를 quaternion으로 변환
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
        
        # Odometry 메시지는 geometry_msgs.msg.Quaternion 타입을 사용해야 함
        # from geometry_msgs.msg import Quaternion
        return Quaternion(qx, qy, qz, qw)
    
    # 움직임 계산
    def compute_movement(self, x_center, distance):
        # 1.거리정보를 이용하여 전진/ 후진 명령을 생성한다.
        # 2.객체의 x좌표를 이용하여 좌/ 우 회전을 제어한다.
        # 이동 명령 생성: 바운딩 박스 중심에 따라 로봇이 이동하도록 계산
        move_cmd = Twist()

        # 1. 거리 유지 명령 생성
        # '가까운, 멀이진
        if distance > self.target_distance + 5:  # 너무 멀면 전진
            move_cmd.linear.x = 0.2  # 앞으로 이동
        #elif distance < self.target_distance - 5:  # 너무 가까우면 후진 # <- 이 부분을 지운다. 
        #    move_cmd.linear.x = -0.2  # 뒤로 이동
        else:
            move_cmd.linear.x = 0.0  # 정지

        # 2. 바운딩 박스의 중심 x 위치에 따라 좌우 회전
        # 바운딩 박스가 치우친 경우, angular을 조절하게 함.
        '''
        예_
        self.frame_width = 640
        center_threshold = self.frame_width * 0.2 = 128 (중앙오차범위)
        화면 중앙 = self.frame_width /2 = 320

        x_center = 100
        x_center < 320 - 128(192) -> true => 왼쪽 회전 
        '''
        center_threshold = self.frame_width * 0.2  # 중앙 오차 허용 범위 (20%) # 테스트 추후에,, 
        if x_center < self.frame_width / 2 - center_threshold:
            move_cmd.angular.z = 0.2  # 왼쪽 회전
        elif x_center > self.frame_width / 2 + center_threshold:
            move_cmd.angular.z = -0.2  # 오른쪽 회전
        else:
            move_cmd.angular.z = 0.0  # 직진

        return move_cmd

def main(args=None):#
    rclpy.init(args=args)
    node = CameraTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
