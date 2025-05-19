import os, xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    arm_pkg = get_package_share_directory('rescue_robot')

    urdf_path = os.path.join(arm_pkg, 'urdf', 'my_robot.urdf.xacro')
    robot_description = xacro.process_file(urdf_path).toxml()
    controllers_file = os.path.join(arm_pkg, 'config', 'arm_control.yaml')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py'))
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'arm_robot'],
        output='screen'
    )

    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_pkg, 'launch', 'spawn_robot.launch.py')
        )
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file],
        output='screen'
    )

    # ✅ joint_state_broadcaster 실행 (ros2_control_node 실행 후)
    joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ros2_control_node,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                )
            ],
        )
    )

    # ✅ joint_trajectory_controller 실행 (joint_state_broadcaster 실행 후)
    joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ros2_control_node,
            on_exit=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_trajectory_controller'],
                    output='screen'
                )
            ],
        )
    )

    single_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    single_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        #gazebo,
        ros2_control_node,
        robot_state_publisher,
        spawn_launch,
        joint_state_broadcaster,
        #joint_trajectory_controller,
        #single_joint_state_broadcaster,
        single_joint_trajectory_controller,
    ])
