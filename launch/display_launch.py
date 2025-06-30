from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('slam_nav2_bot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'slam_nav2_bot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    rviz_file = os.path.join(pkg_path, 'rviz', 'bot_display.rviz')
    
    robot_name = 'slam_bot'
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # Start Gazebo Harmonic
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    # Start bridge between Gazebo and ROS 2
    bridge_params = os.path.join(
        get_package_share_directory('slam_nav2_bot'),
        'params',
        'bridge.yaml'
    )

    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )

    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )


    # Spawn robot entity in simulation
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-x', '0', '-y', '0', '-z', '0.2',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=["-d", rviz_file],
            parameters=[{'use_sim_time': True}]
        ),
        
        spawn_entity,
        start_gazebo_ros_image_bridge_cmd,
        start_gazebo_ros_bridge_cmd,
        gazebo
    ])
