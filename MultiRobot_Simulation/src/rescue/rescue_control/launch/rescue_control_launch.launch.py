import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 네임스페이스 설정 (기본값 없음)
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes (leave empty for no namespace)'
    )
    
    namespace = LaunchConfiguration('namespace')

    openCV_node = Node(
        package='rescue_control',
        executable='camera_openCV_nav2',
        namespace=namespace,
        output='screen',
    )

    GUI_node = Node(
        package='rescue_control',
        executable='GUI',
        namespace=namespace,
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,  # 네임스페이스 인자 선언
        openCV_node,
        GUI_node
    ])
