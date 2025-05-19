from launch import LaunchDescription
from launch_ros.actions import Node
import os, xacro
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    rescue_robot_path = get_package_share_directory('rescue_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    urdf_path = os.path.join(get_package_share_path('rescue_robot'),
                             'urdf', 'my_robot.urdf.xacro')
    rviz_config_path = os.path.join(get_package_share_path('rescue_robot'),
                                    'rviz', 'urdf_config.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    #robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    robot_description = xacro.process_file(urdf_path)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description.toxml()}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    world = os.path.join(
        get_package_share_directory('rescue_turtlebot3_bringup'),
        'worlds',
        'map_wall.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rescue_robot_path, 'launch', 'spawn_robot.launch.py')
        )
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node,
        gzserver_cmd,
        gzclient_cmd,
        spawn_robot_launch
    ])