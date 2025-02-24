import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
from collections import deque

class MapExplorer(Node):
    def __init__(self):
        super().__init__('next_points')

        # /map 구독
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        # /pose 구독 (현재 로봇 위치)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/pose',  # /amcl_pose 대신 /pose 사용
            self.pose_callback,
            10)

        # /nearest_unknown 퍼블리셔
        self.unknown_pub = self.create_publisher(
            PoseStamped,
            '/nearest_unknown',
            10)

        self.map_data = None
        self.map_info = None
        self.robot_grid = None  # 로봇 위치 (grid 좌표)
        self.nearest_unknown = None  # 가장 가까운 미탐색 지역

    def map_callback(self, msg):
        """ /map 데이터 업데이트 및 미탐색 지역 탐색 """
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )

        if self.robot_grid:
            self.nearest_unknown = self.find_nearest_unknown(*self.robot_grid)
            if self.nearest_unknown:
                self.publish_nearest_unknown(self.nearest_unknown)

    def pose_callback(self, msg):
        """ 로봇 위치 (World 좌표) -> Grid 좌표 변환 """
        if self.map_info is None or self.map_data is None:
            self.get_logger().warn("맵 데이터를 아직 수신하지 못했습니다.")
            return

        # 로봇 위치 변환 (World -> Grid)
        x = msg.pose.position.x
        y = msg.pose.position.y
        i, j = self.world_to_grid(x, y)

        if i < 0 or i >= self.map_info.height or j < 0 or j >= self.map_info.width:
            self.get_logger().warn("로봇 위치가 맵 범위를 벗어났습니다.")
            return
        
        self.robot_grid = (i, j)
        self.get_logger().info(f"로봇 Grid 좌표: {self.robot_grid}")

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

    def find_nearest_unknown(self, i, j):
        """ 가장 가까운 -1 좌표 탐색 (BFS 사용) """
        queue = deque([(i, j, 0)])  # (i, j, 거리)
        visited = set()
        visited.add((i, j))

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 상하좌우

        while queue:
            ci, cj, dist = queue.popleft()

            # 미탐색 지역(-1) 발견 시 반환
            if self.map_data[ci, cj] == -1:
                return (ci, cj)

            # 4방향 탐색
            for di, dj in directions:
                ni, nj = ci + di, cj + dj
                if 0 <= ni < self.map_info.height and 0 <= nj < self.map_info.width:
                    if (ni, nj) not in visited:
                        visited.add((ni, nj))
                        queue.append((ni, nj, dist + 1))
        
        return None  # 미탐색 지역 없음

    def publish_nearest_unknown(self, grid_pos):
        """ 가장 가까운 미탐색 지역을 World 좌표로 변환하여 퍼블리시 """
        x, y = self.grid_to_world(*grid_pos)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        self.unknown_pub.publish(msg)
        self.get_logger().info(f"미탐색 지역 발행: {x}, {y} (Grid {grid_pos})")

def main(args=None):
    rclpy.init(args=args)
    node = MapExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
