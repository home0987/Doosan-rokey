import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Transform
from cv_bridge import CvBridge
import cv2, os, time
import numpy as np
import cleaning_robot.lib_img as lib_img
import cleaning_robot.lib_tf as lib_tf
from ament_index_python.packages import get_package_share_directory

class ImageMatcher(Node):
    def __init__(self):
        super().__init__('image_matcher')
        
        self.subscriber = self.create_subscription(
            CompressedImage, '/oakd/rgb/preview/image_raw/compressed', self.image_callback, 10)
        
        self.pnp_tf_pub = self.create_publisher(Transform, 'pnp_trans', 10)
        
        self.bridge = CvBridge()    # CvBridge 초기화 (ROS Image <-> OpenCV 변환)
        # self.orb = cv2.ORB_create(nfeatures=500)    # ORB 특징점 검출기 생성 (찾는 특징점 개수)
        self.sift = cv2.SIFT_create(nfeatures=500)
        
        
        self.bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=False) # BFMatcher (L2 거리 기반)
        
        pkg_path = get_package_share_directory('cleaning_robot')
        img_dir_path = os.path.join(pkg_path, 'imgs')
        ext = os.path.join(img_dir_path, 'ext_orig.png')
        man = os.path.join(img_dir_path, 'man_orig.png')

        # 참조 이미지 로드
        self.ref_images = {
            "ext": cv2.imread(ext, cv2.IMREAD_GRAYSCALE),
            #"man": cv2.imread(man, cv2.IMREAD_GRAYSCALE)
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
            #frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            np_arr = np.frombuffer(msg.data, np.uint8)  # 압축된 데이터 → numpy 배열
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 압축 해제

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

                features = 20 if len(good_matches) <= 20 else len(good_matches)
                ref_matched_pts = np.array([ref_keypoints[m.queryIdx].pt for m in good_matches[:features]], dtype="double")
                cam_matched_pts = np.array([keypoints[m.trainIdx].pt for m in good_matches[:features]], dtype="double")

                # 3D 좌표로 변환 (가정: 사진의 특징점을 월드 좌표에서 특정 위치에 배치)
                points_3D = np.array([
                    (p[0], p[1], 0.0) for p in ref_matched_pts  # 2D → 3D 변환
                ], dtype="double")
                
                R, tvec = lib_tf.pnp(points_3D, cam_matched_pts)
                trans = lib_tf.gen_tf_pnp(R, tvec)
                self.pnp_tf_pub.publish(trans)

                cv2.imshow(f"Matching: {label}", match_img)
                cv2.waitKey(1)
                #time.sleep(2)
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