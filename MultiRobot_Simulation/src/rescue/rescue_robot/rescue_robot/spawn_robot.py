#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# -*- coding: utf-8 -*-
import sys, math
import rclpy
from gazebo_msgs.srv import SpawnEntity


def euler_to_quaternion(roll, pitch, yaw, rad=False):
    """
    오일러 각도(roll, pitch, yaw)를 쿼터니언(qx, qy, qz, qw)으로 변환하는 함수.
    입력 단위는 deg여야 합니다.

    :param roll: X축 회전 (deg여야)
    :param pitch: Y축 회전 (deg여야)
    :param yaw: Z축 회전 (deg여야)
    :return: (qx, qy, qz, qw) 형태의 튜플 (쿼터니언)
    """
    if not rad : roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(yaw)

    # 각도를 절반으로 줄이기
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # 쿼터니언 공식 적용
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_drone')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    content = sys.argv[1]
    namespace = sys.argv[2]

    req = SpawnEntity.Request()
    req.name = namespace
    req.xml = content
    req.robot_namespace = namespace
    req.reference_frame = "world"

    req.initial_pose.position.x = float(-4.0)
    req.initial_pose.position.y = float(-2.0)
    req.initial_pose.position.z = float(0.1)

    req.initial_pose.orientation.z = euler_to_quaternion(0, 0, -90)[2]

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + " " + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
