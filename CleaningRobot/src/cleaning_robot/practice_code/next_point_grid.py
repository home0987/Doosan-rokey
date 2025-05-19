import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import qos_profile_system_default, QoSProfile, ReliabilityPolicy
import numpy as np
from collections import deque
import tf2_ros
import tf2_geometry_msgs  # TF 변환을 위한 패키지

class MapExplorer(Node):
    def __init__(self):
        super().__init__('next_points')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        # TF Listener 생성
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

        # /nearest_unknown 퍼블리셔
        self.unknown_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/nearest_unknown',
            10)

        self.map_data = None
        self.map_info = None
        self.robot_grid = None  # 로봇 위치 (grid 좌표)
        self.nearest_unknown = None  # 가장 가까운 미탐색 지역
        self.visited = set()

        #self.timer = self.create_timer(1.0, point)

    def map_callback(self, msg):
        """ /map 데이터 업데이트 및 미탐색 지역 탐색 """
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

        self.get_logger().info("/map received")

        if self.robot_grid:
            self.nearest_unknown = self.find_nearest_unknown(*self.robot_grid)
            if self.nearest_unknown:
                self.publish_nearest_unknown(self.nearest_unknown)

    def odom_callback(self, msg):
        """ 로봇 위치 (World 좌표) -> Grid 좌표 변환 """
        if self.map_info is None or self.map_data is None:
            self.get_logger().warn("맵 데이터를 아직 수신하지 못했습니다.")
            return

        # /odom 좌표 가져오기
        odom_pose = PoseStamped()
        odom_pose.header = msg.header
        odom_pose.pose = msg.pose.pose

        try:
            # TF 변환 요청 (odom -> map)
            transform = self.tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())

            # 좌표 변환 수행
            map_pose = tf2_geometry_msgs.do_transform_pose(odom_pose, transform)
            x, y = map_pose.pose.position.x, map_pose.pose.position.y

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

    def find_nearest_unknown(self, i, j, range_num=3):
        """ 가장 가까운 -1 좌표 탐색 (BFS 사용) """
        self.get_logger().info(f"point shearch started range = {range_num}")
        #현재 위치 기준 그리드 크기
        i_low = i-range_num if i-range_num>=0 else 0
        j_low = j-range_num if j-range_num>=0 else 0
        i_high = i+range_num if i+range_num<self.map_info.height else self.map_info.height
        j_high = j+range_num if j+range_num<self.map_info.width else self.map_info.width
        if not i_low and not j_low and i_high == self.map_info.height and j_high == self.map_info.width:
            self.get_logger().info(f"point shearch ended range = {range_num}")
            return

        points = list(zip(*np.where(self.map_data[i_low:i_high, j_low:j_high] == -1)))   # 미개척 좌표
        for p in points:
            if np.any(self.map_data[p[0]-1:p[0]+1, p[1]-1:p[1]+1] == 0) and p not in self.visited:    # -1/0 붙어있는 좌표
            #if p not in self.visited:
                self.visited.add((p))
                return p
            
        self.find_nearest_unknown(i, j, range_num+2)


    def publish_nearest_unknown(self, grid_pos):
        """ 가장 가까운 미탐색 지역을 World 좌표로 변환하여 퍼블리시 """
        x, y = self.grid_to_world(*grid_pos)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        self.unknown_pub.publish(msg)
        self.nearest_unknown = None
        self.get_logger().info(f"미탐색 지역 발행: {x}, {y} (Grid {grid_pos})")

def main(args=None):
    rclpy.init(args=args)
    node = MapExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
