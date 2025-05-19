import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.navigator = BasicNavigator()
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/pose', self.pose_callback, 10)
        self.timer = self.create_timer(3.0, self.goal_callback)
        self.goal = None
        self.map_data = None
        self.pose = None

    def map_callback(self, msg):
        """SLAM으로 생성된 지도 업데이트 및 Frontier 탐색"""

        if self.pose is None:
            self.get_logger().warn("Waiting for robot pose before processing map.")
            return
        
        width, height = msg.info.width, msg.info.height
        self.map_data = np.array(msg.data).reshape((height, width))

        # 1. 도달 가능한 지역만 추출 (Flood Fill)
        reachable_area = self.get_reachable_area(self.map_data, msg.info) 

        # 2. 미탐색 지역에서 Frontier 찾기
        unexplored_map = (self.map_data == -1).astype(np.uint8) * 255  # 미탐색 지역 (Unknown)
        frontiers = self.find_frontiers(unexplored_map, reachable_area)

        if frontiers:
            self.goal = self.select_goal(frontiers, msg.info)

    def goal_callback(self):
            if self.goal:
                self.navigator.goToPose(self.goal)

    def pose_callback(self, msg):
        self.pose = msg.pose.pose
        # self.get_logger().info(f"robot_pose: {self.pose}")

    def get_reachable_area(self, map_data, msg):
        """Flood Fill을 사용하여 로봇이 도달 가능한 영역만 식별"""
        height, width = map_data.shape
        reachable = np.zeros_like(map_data, dtype=np.uint8)

        # 로봇의 현재 위치 가져오기
        robot_pose = self.pose

        # self.get_logger().info(f"robot_pose: {robot_pose}")
        # self.get_logger().info(f"map_data: {map_data}")

        start_x = int((robot_pose.position.x - msg.origin.position.x) / msg.resolution) 
        start_y = int((robot_pose.position.y - msg.origin.position.y) / msg.resolution)

        # BFS Flood Fill
        queue = [(start_x, start_y)]
        visited = set(queue)

        while queue:
            x, y = queue.pop(0)
            reachable[y, x] = 255  # 도달 가능한 영역

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if (nx, ny) not in visited and map_data[ny, nx] != 100:  # 장애물이 아니면 이동 가능
                        queue.append((nx, ny))
                        visited.add((nx, ny))

        return reachable

    def find_frontiers(self, unexplored_map, reachable_area):
        """Canny Edge Detection을 사용하여 Frontier 찾기 (도달 가능한 영역 필터링)"""
        edges = cv2.Canny(unexplored_map, 50, 100)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        frontiers = []
        for contour in contours:
            if len(contour) >= 1:  # 작은 컨투어 제거
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # 도달 가능한 영역인지 확인
                    if 0 <= cx < reachable_area.shape[1] and 0 <= cy < reachable_area.shape[0]:
                        if reachable_area[cy, cx] == 255 and unexplored_map[cy, cx] == 255:  
                            # 탐색되지 않은 지역(Unknown)과 맞닿은 영역만 추가
                            frontiers.append((cx, cy))

        return frontiers

    def select_goal(self, frontiers, map_info):
        """가장 가까운 Frontier를 선택하여 Goal로 설정"""
        robot_pose = self.pose
        
        MIN_GOAL_DISTANCE = 1.0
        min_dist = float('inf')
        best_goal = None

        for fx, fy in frontiers:
            wx = map_info.origin.position.x + fx * map_info.resolution
            wy = map_info.origin.position.y + fy * map_info.resolution

            dist = np.sqrt((wx - robot_pose.position.x)**2 + (wy - robot_pose.position.y)**2)

            if dist < MIN_GOAL_DISTANCE:
                continue  # 너무 가까우면 건너뜀

            if dist < min_dist:
                min_dist = dist
                best_goal = (wx, wy)

        if best_goal:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = best_goal[0]
            goal_pose.pose.position.y = best_goal[1]
            goal_pose.pose.orientation.w = 1.0

            return goal_pose

        return None

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
