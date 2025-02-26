import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import cleaning_robot.lib as lib

class ImageMatcher(Node):
    def __init__(self):
        super().__init__('image_matcher')
        
        self.subscriber = self.create_subscription(
            Image, '/oakd/rgb/preview/image_raw', self.image_callback, 10)
        
        # CvBridge 초기화 (ROS Image <-> OpenCV 변환)
        self.bridge = CvBridge()
        
        # ORB 특징점 검출기 생성 
        # ORB가 SIFT보다 SLAM에 적합하대용
        self.orb = cv2.ORB_create(nfeatures=500) # 찾는 특징점 개수
        
        # BFMatcher (Hamming 거리 기반)
        # FLANN보다 느린데 정확도 높음 (빠른 매칭 가능~)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # FLANN 기반 매칭을 위한 파라미터 설정
        # FLANN_INDEX_LSH = 6  # ORB는 이진(Binary) 디스크립터이므로 LSH 사용
        # index_params = dict(algorithm=FLANN_INDEX_LSH,
        #                     table_number=6,  # LSH 테이블 개수
        #                     key_size=12,     # 키 크기
        #                     multi_probe_level=1)  # 검색 깊이
        # search_params = dict(checks=50)  # 검색 반복 횟수

        # # FLANN Matcher 생성
        # self.flann = cv2.FlannBasedMatcher(index_params, search_params)
        
        # 참조 이미지 로드
        self.ref_images = {
            "ext": cv2.imread("src/cleaning_robot/config/ext_orig.png", cv2.IMREAD_GRAYSCALE),
            "man": cv2.imread("src/cleaning_robot/config/man_orig.png", cv2.IMREAD_GRAYSCALE)
        }

        # 참조 이미지에서 ORB 특징점 계산
        self.ref_keypoints = {}
        self.ref_descriptors = {}
        
        for key, img in self.ref_images.items():
            if img is not None:
                kp, des = self.orb.detectAndCompute(img, None)
                self.ref_keypoints[key] = kp
                self.ref_descriptors[key] = des
            else:
                self.get_logger().error(f"Failed to load reference image: {key}")
        
        # 이전 프레임 저장용 변수
        self.prev_keypoints = None
        self.prev_descriptors = None
        self.prev_frame = None

    def image_callback(self, msg):
        """ OAK-D 카메라의 raw 이미지를 받아 처리 """
        try:
            # ROS Image → OpenCV BGR 이미지 변환
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # ORB 특징점 검출 및 디스크립터 계산
            keypoints, descriptors = self.orb.detectAndCompute(gray_frame, None)

            # if self.prev_frame is not None:
            #     # BFMatcher를 사용하여 이전 프레임과 현재 프레임 매칭
            #     matches = self.bf.match(self.prev_descriptors, descriptors)

            #     # 매칭 결과 정렬 (거리 기준으로 정렬)
            #     matches = sorted(matches, key=lambda x: x.distance)
                
            # if self.prev_frame is not None and self.prev_descriptors is not None:
            #     # FLANN을 사용한 KNN 매칭 (k=2)
            #     matches = self.flann.knnMatch(self.prev_descriptors, descriptors, k=2)

            #     # Lowe's ratio test 적용하여 좋은 매칭 선택
            #     good_matches = []
            #     for m, n in matches:
            #         if m.distance < 0.75 * n.distance:  # 0.75 비율로 설정
            #             good_matches.append(m)
            

                # 상위 N개만 선택 (매칭 결과 과부하 방지)
                # num_good_matches = min(50, len(matches))
                # good_matches = matches[:num_good_matches]

                # # 매칭된 결과를 시각화
                # match_img = cv2.drawMatches(
                #     self.prev_frame, self.prev_keypoints,
                #     gray_frame, keypoints,
                #     good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                
            if descriptors is not None:
                for label in self.ref_keypoints.keys():
                    ref_keypoints = self.ref_keypoints[label]
                    ref_descriptors = self.ref_descriptors[label]

                    if ref_descriptors is None or descriptors is None:
                        continue  # 디스크립터가 없는 경우 패스
                    
                    matches = self.bf.match(ref_descriptors, descriptors)
                    matches = sorted(matches, key=lambda x: x.distance)
                    good_matches = matches[:50]  # 상위 50개 선택
                    
                    match_img = cv2.drawMatches(
                        self.ref_images[label], ref_keypoints,
                        gray_frame, keypoints,
                        good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

                    cv2.imshow(f"Matching: {label}", match_img)
                    cv2.waitKey(1)
                # lib.img_matching(match_img) # 일케?

            # 현재 프레임을 이전 프레임으로 저장
            self.prev_frame = gray_frame
            self.prev_keypoints = keypoints
            self.prev_descriptors = descriptors

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageMatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()