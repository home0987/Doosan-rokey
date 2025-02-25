import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_launch_dir = os.path.join(get_package_share_directory('turtlebot4_navigation'), 'launch')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    nav2_yaml_dir = os.path.join(
        get_package_share_directory('cleaning_robot'),
        'config',
        'nav2_yj.yaml'
    )

    slam_yaml_dir = os.path.join(
        get_package_share_directory('cleaning_robot'),
        'config',
        'slam_yj.yaml'
    )
    
    nav2_yaml_dir = LaunchConfiguration('nav2_yaml_dir', default=nav2_yaml_dir)
    slam_yaml_dir = LaunchConfiguration('slam_yaml_dir', default=nav2_yaml_dir)

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'nav2.launch.py')
        ),
        launch_arguments={'params_file': nav2_yaml_dir}.items()
    )

    slam_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'slam.launch.py')
        ),
        launch_arguments={'params':  slam_yaml_dir}.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(nav2_cmd)
    ld.add_action(slam_cmd)

    return ld