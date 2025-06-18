from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
import os
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # File paths
    pkg_path = get_package_share_directory('slam_nav2_bot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'slam_nav2_bot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')

    robot_name = 'slam_bot'
    robot_namespace = ''

    # Convert XACRO to URDF
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # Launch Gazebo Ignition
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_file],
        output='screen'
    )

    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher"
    )

    
    

    # Spawn the robot in Ignition
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

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_node = Node(
        package="rviz2",
        executable= "rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d", os.path.join(
                get_package_share_directory("slam_nav2_bot"),
                "rviz",
                "display.rviz"
            )
        ]
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        joint_state_publisher,
        rviz_node,
        robot_state_publisher
    ])
