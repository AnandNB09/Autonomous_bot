from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # File paths
    pkg_path = get_package_share_directory('slam_nav2_bot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'slam_nav2_bot.urdf.xacro')
    world_file = os.path.join(pkg_path, 'worlds', 'empty.world')
    rviz_file = os.path.join(pkg_path, 'rviz', 'display.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_name = 'slam_bot'

    # Convert XACRO to URDF
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

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

    # Joint state publisher GUI
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher",
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )

    # RViz2 visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True'),
        gazebo,
        start_gazebo_ros_bridge_cmd,
        start_gazebo_ros_image_bridge_cmd,
        spawn_entity,
        joint_state_publisher,
        robot_state_publisher,
        rviz_node
    ])
