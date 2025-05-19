from launch import LaunchDescription
from launch_ros.actions import Node
import os, xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_path, get_package_share_directory

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('rescue_robot'),
                             'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('rescue_robot'),
                                    'rviz', 'urdf_config.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    robot_description = xacro.process_file(urdf_path)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description.toxml()}]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{'use_sim_time': use_sim_time, "use_gui": False}]
    )

    # ✅ arm_base_link가 base_link에 연결되지 않은 경우, TF를 추가해 해결
    static_tf_publisher_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "arm_base_link"],
        output="screen"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        #robot_state_publisher_node,
        joint_state_publisher_node,
        static_tf_publisher_node,
        rviz2_node,
    ])