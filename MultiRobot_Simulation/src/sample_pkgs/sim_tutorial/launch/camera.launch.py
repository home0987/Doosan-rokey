import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = "sim_tutorial"

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot_5.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    my_robot_description_pkg = FindPackageShare('sim_tutorial').find('sim_tutorial')

    # Use PathJoinSubstitution to construct the full path to the world file
    world_file_path = PathJoinSubstitution([
        my_robot_description_pkg,
        'worlds',
        'with_robot.world'
    ])

#    world_file_path = PathJoinSubstitution(
#        [package_name, 'worlds', 'construction.world']
#    )

    # Declare the world file as a launch argument
    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_file_path,
        #default_value='src/sim_tutorial/worlds/with_robot.world',
        description='Full path to the world model file to load'
    )


    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot"],
        output="screen",
    )

    # Launch them all!
    return LaunchDescription(
        [
            declare_world_cmd,
            rsp,
            gazebo,
            spawn_entity,
        ]
    )
