import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import math
from geometry_msgs.msg import Pose, Point, Quaternion

# Set model pose
initial_pose = Pose()
#initial_pose.position = Point(2.0, 3.0, 0.5)  # x, y, z
    
# For a simple rotation around Z axis (yaw = 1.57 rad = 90 degrees)
yaw = 1.57
#initial_pose.orientation = Quaternion(
#    0.0,  # x
#    0.0,  # y
#    math.sin(yaw/2),  # z
#    math.cos(yaw/2)   # w
#)

def generate_launch_description():
    package_name = "sim_tutorial"

    rsp3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot_3.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    rsp4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(package_name), "launch", "robot_4.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")]
        ),
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity3 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot3","-robot_namespace","robot3"],
        output="screen",
    )

    spawn_entity4 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "with_robot4", "-robot_namespace","robot4","-x","1.0","-y","2.0"],
        output="screen",
    )

    # Launch them all! => add list...
    return LaunchDescription(
        [
            rsp3,
            rsp4,
            gazebo,
            spawn_entity3,
            spawn_entity4,
        ]
    )


#def generate_launch_description():
#    ld = LaunchDescription()
#    talker_node = Node(
#        package="demo_nodes_cpp",
#        executable="talker",
#    )
#    listener_node = Node(
#        package="demo_nodes_py",
#        executable="listener"
#    )
#    ld.add_action(talker_node)
#    ld.add_action(listener_node)
#    return ld
