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
            Image, '/oakd/rgb/preview/image_raw/compressed', self.image_callback, 10)
        
        self.bridge = CvBridge()    # CvBridge 초기화 (ROS Image <-> OpenCV 변환)
        # self.orb = cv2.ORB_create(nfeatures=500)    # ORB 특징점 검출기 생성 (찾는 특징점 개수)
        self.sift = cv2.SIFT_create(nfeatures=500)
        
        
        self.bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False) # BFMatcher (L2 거리 기반)
        
        # 참조 이미지 로드
        self.ref_images = {
            "ext": cv2.imread("/home/rokey/cleaning_robot/src/cleaning_robot/config/ext_orig.png", cv2.IMREAD_GRAYSCALE),
            "man": cv2.imread("/home/rokey/cleaning_robot/src/cleaning_robot/config/man_orig.png", cv2.IMREAD_GRAYSCALE)
        }

        # 참조 이미지에서 ORB 특징점 계산
        self.ref_keypoints = {}
        self.ref_descriptors = {}
        
        for key, img in self.ref_images.items():
            if img is not None:
                # kp, des = self.orb.detectAndCompute(img, None)
                kp, des = self.sift.detectAndCompute(img, None)     # np.float32
                self.ref_keypoints[key] = kp
                self.ref_descriptors[key] = des
            else:
                self.get_logger().error(f"Failed to load reference image: {key}")
        
        # 이전 프레임 저장용 변수
        self.prev_keypoints = None
        self.prev_descriptors = None
    
    def image_callback(self, msg):
        try:
            # ROS Image → OpenCV BGR 이미지 변환
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            keypoints, descriptors = self.sift.detectAndCompute(gray_frame, None)

            if descriptors is None or len(descriptors) < 2:
                self.get_logger().warn("Not enough descriptors found, skipping frame.")
                return

            for label in self.ref_keypoints.keys():
                ref_keypoints = self.ref_keypoints[label]
                ref_descriptors = self.ref_descriptors[label]

                if ref_descriptors is None or len(ref_descriptors) < 2:
                    continue
                
                matches = self.bf.knnMatch(ref_descriptors, descriptors, k=2)
                
                good_matches = []
                for match_pair in matches:
                    if len(match_pair) == 2:
                        m, n = match_pair
                        if m.distance < 0.85 * n.distance:
                            good_matches.append(m)
                
                match_img = cv2.drawMatches(
                    self.ref_images[label], ref_keypoints,
                    gray_frame, keypoints,
                    good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
                
                # if self.prev_keypoints is not None and self.prev_descriptors is not None:
                #     prev_pts = np.float32([self.prev_keypoints[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
                #     curr_pts = np.float32([keypoints[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

                #     # 오버레이에 라인(매칭선) 그리기
                #     for p0, p1 in zip(prev_pts, curr_pts):
                #         pt0 = tuple(map(int, p0.ravel()))
                #         pt1 = tuple(map(int, p1.ravel()))
                #         cv2.line(self.overlay, pt0, pt1, (0, 0, 255), 2)

                # 이번 프레임을 다음 프레임을 위한 prev_*에 저장
                self.prev_keypoints = keypoints
                self.prev_descriptors = descriptors
                
                # 키포인트 표시
                for match in good_matches:
                    x, y = map(int, keypoints[match.trainIdx].pt)
                    cv2.circle(gray_frame, (x, y), 5, (0, 255, 0), -1)
                
                cv2.imshow(f"Matching: {label}", match_img)
                cv2.waitKey(1)
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