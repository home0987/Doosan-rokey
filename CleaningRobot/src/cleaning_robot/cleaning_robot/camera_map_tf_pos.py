import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import qos_profile_system_default, QoSProfile, ReliabilityPolicy
import numpy as np
from collections import deque
import tf2_ros
import tf2_geometry_msgs  # TF 변환을 위한 패키지
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Transform, Pose
import cleaning_robot.lib_tf as lib_tf
import cleaning_robot.lib_img as lib_img

class MapExplorer(Node):
    def __init__(self):
        super().__init__('img_map_pos')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # TF Listener 생성
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)
        self.current_pos_marker = self.create_publisher(Marker, 'current_pos_marker', 10)

        # /map 구독
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)

        # /pnp_trans 퍼블리셔
        self.pnp_trans_sub = self.create_subscription(
            Transform,
            '/pnp_trans',
            self.pnp_callback,
            10)

        self.map_data = None
        self.map_info = None
        self.robot_current_pos = None
        self.robot_grid = None  # 로봇 위치 (grid 좌표)
        self.pnp_mat = None  # 가장 가까운 미탐색 지역
        self.pose = None

        #self.timer = self.create_timer(1.0, point)

    def map_callback(self, msg):
        """ /map 데이터 업데이트 및 미탐색 지역 탐색 """
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

        self.get_logger().info("/map received")

        #lib_img.visual_map_topic(self.map_info.height, self.map_info.width, self.map_data, self.robot_grid)

    def pose_callback(self, msg):
        self.pose = msg.pose.pose

    def world_to_grid(self, x, y):
        """ World 좌표 -> Grid 좌표 변환 """
        j = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        i = int(self.map_info.height - (y - self.map_info.origin.position.y) / self.map_info.resolution)
        return i, j

    def grid_to_world(self, i, j):
        """ Grid 좌표 -> World 좌표 변환 """
        x = j * self.map_info.resolution + self.map_info.origin.position.x
        y = (self.map_info.height - i) * self.map_info.resolution + self.map_info.origin.position.y
        return x, y

    def pnp_callback(self, transform_msg):
        self.pnp_mat = transform_msg
        if self.robot_grid:
            #x, y = self.grid_to_world(*self.robot_grid)

            camera_base = Pose()
            camera_base.position.x = self.pose[0]
            camera_base.position.y = self.pose[1]
            camera_base.position.z = self.pose[2]

            self.transform_camera_base = self.tf_buffer.lookup_transform("oakd_rgb_camera_optical_frame", "base_link", rclpy.time.Time())
            pre_pos = tf2_geometry_msgs.do_transform_pose(camera_base, self.transform_camera_base)

            # tvec = lib_tf.tf_cam_2_base(x, y, 0)
            # self.get_logger().info(f"tvec = {tvec}")
            # x, y = tvec[0][0], tvec[0][1]

            x, y, z = pre_pos.position.x, pre_pos.position.y, pre_pos.position.z
            marker_current = Marker()
            marker_current = lib_img.get_marker(x, y, 0.0, self.get_clock().now().to_msg(), 0.0, 0.0, 1.0)
            self.current_pos_marker.publish(marker_current)

            T = lib_tf.transform_to_matrix(transform_msg)

            # 동차 좌표계로 변환 (z=0, w=1)
            point = np.array([x, y, z, 1])

            # 변환 행렬 적용
            transformed_point = T @ point

            # 새로운 (x, y) 좌표
            x_new, y_new = transformed_point[0], transformed_point[1]
            x_new_map, y_new_map = self.world_to_grid(x_new, y_new)
            
            # x_point = point[:3].transpose() + 0.01*T[:3, 3] * self.map_info.resolution
            # x_new, y_new = x_point[:2]

            marker = Marker()
            marker = lib_img.get_marker(float(x_new), float(y_new), 0.0, self.get_clock().now().to_msg())
            self.marker_pub.publish(marker)
            self.get_logger().info(f"robot pos x = {self.robot_current_pos[0]}, y = {self.robot_current_pos[1]}")
            # 변환된 (x, y) 값을 로그로 출력
            self.get_logger().info(f"Transformed (x, y): ({x_new:.3f}, {y_new:.3f})")


def main(args=None):
    rclpy.init(args=args)
    node = MapExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
