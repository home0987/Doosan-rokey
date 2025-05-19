import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

class CamToBaseTransformer(Node):
    def __init__(self):
        super().__init__('cam_to_base_transformer')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'tvec_camera',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        self.R_bc = np.array([
            [0.000,  0.000,  1.000],
            [-1.000, 0.000,  0.000],
            [0.000, -1.000,  0.000]
        ])

        self.t_bc = np.array([
            [-0.060],
            [0.000],
            [0.244]
        ])

    def listener_callback(self, msg):
        tvec_camera = np.array(msg.data).reshape(3, 1)
        tvec_base = np.dot(self.R_bc, tvec_camera) + self.t_bc

        self.get_logger().info(f"Received tvec_camera: {tvec_camera.flatten()}")
        self.get_logger().info(f"Transformed tvec_base: {tvec_base.flatten()}")

def main(args=None):
    rclpy.init(args=args)
    node = CamToBaseTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
