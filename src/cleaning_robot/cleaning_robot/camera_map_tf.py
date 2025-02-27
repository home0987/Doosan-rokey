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

        # /map 구독
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        # /pose 구독 (현재 로봇 위치)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=qos_profile
        )#callback_group=ReentrantCallbackGroup())

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
        #self.transform_camera_base = self.tf_buffer.lookup_transform("oakd_rgb_camera_optical_frame", "base_link", rclpy.time.Time())

        #self.timer = self.create_timer(1.0, point)

    def map_callback(self, msg):
        """ /map 데이터 업데이트 및 미탐색 지역 탐색 """
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

        self.get_logger().info("/map received")

        #lib_img.visual_map_topic(self.map_info.height, self.map_info.width, self.map_data, self.robot_grid)

    def odom_callback(self, msg):
        """ 로봇 위치 (World 좌표) -> Grid 좌표 변환 """
        if self.map_info is None or self.map_data is None:
            self.get_logger().warn("맵 데이터를 아직 수신하지 못했습니다.")
            return

        # # /odom 좌표 가져오기
        # odom_pose = PoseStamped()
        # odom_pose.header = msg.header
        # odom_pose.pose = msg.pose.pose
        # self.get_logger().info(f"odom_pos.pos = {odom_pose.pose}")
        # Odometry().pose.pose
        #PoseStamped().pose.position
        try:
            # TF 변환 요청 (odom -> map)
            transform = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())

            # 변환 행렬 추출
            trans = transform.transform.translation  # 변위 (Translation)
            rot = transform.transform.rotation       # 회전 (Rotation - Quaternion)

            # Odometry 좌표
            odom_x = msg.pose.pose.position.x
            odom_y = msg.pose.pose.position.y
            odom_z = msg.pose.pose.position.z

            # 쿼터니언 -> 회전 행렬 변환
            rotation_matrix = lib_tf.quaternion_to_rotation_matrix(rot.x, rot.y, rot.z, rot.w)

            # 변환 적용 (R * odom + T)
            odom_position = np.array([odom_x, odom_y, odom_z])  # 3x1 벡터
            translation = np.array([trans.x, trans.y, trans.z])  # 3x1 벡터
            map_position = np.dot(rotation_matrix, odom_position) + translation  # 좌표 변환 수행

            # 새로운 쿼터니언 적용 (회전 변환)
            odom_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            odom_rotation_matrix = lib_tf.quaternion_to_rotation_matrix(*odom_quat)
            map_rotation_matrix = np.dot(rotation_matrix, odom_rotation_matrix)
            map_quat = lib_tf.quaternion_from_matrix(map_rotation_matrix)
            self.robot_current_pos = map_position
            x, y = map_position[0], map_position[1]

            # 변환된 좌표 출력
            self.get_logger().info(f"map 좌표: x={map_position[0]}, y={map_position[1]}")


            # 좌표 변환 수행
            # map_pose = tf2_geometry_msgs.do_transform_pose(odom_pose.pose, transform)
            # self.get_logger().info(f"map_pose = {map_pose}")
            # x, y = map_pose.position.x, map_pose.position.y

            # 변환된 좌표를 Grid로 변환
            i, j = self.world_to_grid(x, y)

            if i < 0 or i >= self.map_info.height or j < 0 or j >= self.map_info.width:
                self.get_logger().warn("로봇 위치가 맵 범위를 벗어났습니다.")
                return

            self.robot_grid = (i, j)
            self.get_logger().info(f"로봇 Grid 좌표 (map 기준): {self.robot_grid}")

        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")

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
            camera_base.position.x = self.robot_current_pos[0]
            camera_base.position.y = self.robot_current_pos[1]
            camera_base.position.z = self.robot_current_pos[2]

            self.transform_camera_base = self.tf_buffer.lookup_transform("oakd_rgb_camera_optical_frame", "base_link", rclpy.time.Time())
            pre_pos = tf2_geometry_msgs.do_transform_pose(camera_base, self.transform_camera_base)

            # tvec = lib_tf.tf_cam_2_base(x, y, 0)
            # self.get_logger().info(f"tvec = {tvec}")
            # x, y = tvec[0][0], tvec[0][1]

            x, y, z = pre_pos.position.x, pre_pos.position.y, pre_pos.position.z

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
