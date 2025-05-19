# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import math
import xacro

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def euler_to_quaternion(roll, pitch, yaw, rad=False):
    """
    오일러 각도(roll, pitch, yaw)를 쿼터니언(qx, qy, qz, qw)으로 변환하는 함수.
    입력 단위는 deg여야 합니다.

    :param roll: X축 회전 (deg 단위)
    :param pitch: Y축 회전 (deg 단위)
    :param yaw: Z축 회전 (deg 단위)
    :param rad: radian 단위 사용 여부 (기본값: False)
    :return: (qx, qy, qz, qw) 형태의 튜플 (쿼터니언)
    """
    if not rad:
        roll, pitch, yaw = map(math.radians, [roll, pitch, yaw])

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def generate_launch_description():
    # URDF 파일 경로 가져오기
    rescue_robot_pkg_path = get_package_share_path('rescue_robot')
    xacro_path = rescue_robot_pkg_path / 'urdf' / 'my_robot.urdf.xacro'

    # Xacro 파일을 URDF로 변환
    robot_description = xacro.process_file(str(xacro_path)).toxml()

    # 초기 위치 및 자세 설정
    x_pose = LaunchConfiguration('x_pose', default='-4.0')
    y_pose = LaunchConfiguration('y_pose', default='-2.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')

    x_rad = LaunchConfiguration('x_rad', default='0.0')
    y_rad = LaunchConfiguration('y_rad', default='0.0')
    z_rad = LaunchConfiguration('z_rad', default=str(math.radians(-90.0)))

    # Xacro 변환 수행 (변환된 URDF를 임시 파일로 저장)
    urdf_raw = xacro.process_file(str(xacro_path)).toxml()
    urdf_file_path = os.path.join('/tmp', 'robot.urdf')

    with open(urdf_file_path, 'w') as urdf_file:
        urdf_file.write(urdf_raw)

    # Gazebo 로봇 스폰
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'arm_robot',
            '-file', urdf_file_path,
            '-x', x_pose, '-y', y_pose, '-z', z_pose,
            '-R', x_rad, '-P', y_rad, '-Y', z_rad
        ],
        output='screen'
    )

    # Launch Description 생성
    ld = LaunchDescription([
        DeclareLaunchArgument('x_pose', default_value='-4.0', description='Initial x position'),
        DeclareLaunchArgument('y_pose', default_value='-2.0', description='Initial y position'),
        DeclareLaunchArgument('z_pose', default_value='0.1', description='Initial z position'),
        DeclareLaunchArgument('x_rad', default_value='0.0', description='Initial roll rotation'),
        DeclareLaunchArgument('y_rad', default_value='0.0', description='Initial pitch rotation'),
        DeclareLaunchArgument('z_rad', default_value=str(math.radians(-90.0)), description='Initial yaw rotation'),
        spawn_entity
    ])

    return ld
