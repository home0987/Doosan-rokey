import os
import rclpy
import random
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory

# 제외할 좌표 범위 리스트
EXCLUDED_RANGES = [
    ((-6.0, -2.8), (1.6, 4.7)),  # 첫 번째 제외 범위 (x 범위, y 범위)
    ((-2.5, 0.9), (1.3, 4.5)),  # 두 번째 제외 범위
    ((-6.0, -2.8), (-1.7, 1.0)), # 세 번째 제외 범위
    ((-2.9, 1.0), (-0.2, 1.2))   # 네 번째 제외 범위
]

def is_excluded(x, y):
    """해당 좌표 (x, y)가 제외할 범위 내에 있는지 확인"""
    for x_range, y_range in EXCLUDED_RANGES:
        if x_range[0] <= x <= x_range[1] and y_range[0] <= y <= y_range[1]:
            return True
    return False

def get_random_pose():
    while True:
        x = random.uniform(-6.0, 1.0)
        y = random.uniform(-2.0, 5.0)
        
        # 제외할 범위에 속하지 않는 경우 좌표 반환
        if not is_excluded(x, y):
            return x, y

def main():
    # Start node
    sdf_pose_x, sdf_pose_y = get_random_pose()
    sdf_pose_z = 0.1

    rclpy.init()
    node = rclpy.create_node("entity_spawner")

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, "/spawn_entity")

    node.get_logger().info("Connecting to `/spawn_entity` service...")
    
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...connected!")
    
    # script_dir = os.path.dirname(os.path.realpath(__file__))
    # sdf_file_path = os.path.join(script_dir, "models/model.sdf") 
    script_dir = os.path.join(get_package_share_directory('rescue_turtlebot3_bringup'), 'models')
    sdf_file_path = os.path.join(script_dir, "hyu_robot.sdf") 

    # Set data for request
    request = SpawnEntity.Request()
    request.name = "jun_robot"
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = "jun_robot"
    request.initial_pose.position.x = float(sdf_pose_x)
    request.initial_pose.position.y = float(sdf_pose_y)
    request.initial_pose.position.z = float(sdf_pose_z)

    node.get_logger().info("Sending service request to `/spawn_entity`")

    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()