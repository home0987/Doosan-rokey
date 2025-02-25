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

    next_point_node = Node(
        package='cleaning_robot',
        executable='next_point_BFS',
        namespace=namespace,
        output='screen',
    )

    move_node = Node(
        package='cleaning_robot',
        executable='move_goal',
        namespace=namespace,
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,  # 네임스페이스 인자 선언
        next_point_node,
        move_node
    ])