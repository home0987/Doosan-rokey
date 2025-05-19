import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

import launch
import launch_ros.actions
import ament_index_python.packages

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_path = os.path.join(get_package_share_directory("urdf_tutorial"))
    xacro_file = os.path.join(pkg_path, "urdf", "robot_2.xacro")
    robot_description = xacro.process_file(xacro_file)
    params = {"robot_description": robot_description.toxml(), "use_sim_time": use_sim_time}


    rviz_default_path = os.path.join(
        ament_index_python.packages.get_package_share_directory('urdf_tutorial'),
        'config', 'default.rviz'
    )


    return LaunchDescription(
    [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="use sim time"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[params],
            ),
            launch.actions.DeclareLaunchArgument(
                'rviz_config',
                default_value=rviz_default_path,
                description='Full path to the RViz config file'
            ),
            Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[
                {'use_sim_time': True}
            ]
            ),
            ]
    )