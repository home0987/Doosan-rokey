import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header  # Header 추가
import time

class ArmPositionController(Node):
    def __init__(self):
        super().__init__('arm_position_controller')
        self.publisher_ = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)
        time.sleep(2)  # Ensure the publisher is fully set up

        # 조인트 위치 변경을 위한 상태 변수
        self.positions_list = [[0.0, 0.0], [0.5, -0.5], [0.7, 0.3]]  # 여러 위치 지정 가능
        self.current_index = 0  # 현재 위치 인덱스
        
        # 1초마다 `move_joints` 실행
        self.timer = self.create_timer(2.0, self.move_joints)

    def move_joints(self):
        # 현재 위치 선택
        positions = self.positions_list[self.current_index]
        
        # JointTrajectory 메시지 생성
        msg = JointTrajectory()
        msg.header = Header()
        #msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'  # 기준 프레임 지정

        msg.joint_names = ['arm_base_forearm_joint', 'forearm_hand_joint']
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 2  # Move in 2 seconds
        
        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Moving joints to positions: {positions}')

        # 다음 위치로 업데이트
        self.current_index = (self.current_index + 1) % len(self.positions_list)

def main(args=None):
    rclpy.init(args=args)
    controller = ArmPositionController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
