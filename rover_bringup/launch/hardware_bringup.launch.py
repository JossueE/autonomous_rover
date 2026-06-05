import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    log_level = LaunchConfiguration('log_level')

    pkg_robot_core = get_package_share_directory('robot_core')

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_core, 'launch', 'robot_state_publisher.launch.py')
        )
    )

    wheels_driver_node = Node(
        package='zlac8015d_driver2_cpp',
        executable='wheels_driver',
        name='wheels_driver',
        output='screen',
        arguments=[LaunchConfiguration('motors_port')],
        ros_arguments=['--log-level', log_level],
        parameters=[{
            'unlock_driver': True,
            'accel_time_ms': 500,
            'decel_time_ms': 500,
            'wheels_separation': 0.35,
            'resolution_mode': True
        }]
    )

    twist_mux_node = Node(
        package='rover_bringup',
        executable='twist_priority_mux.py',
        name='twist_priority_mux',
        output='screen',
        ros_arguments=['--log-level', log_level],
        parameters=[{
            'high_topic': '/cmd_vel_safe',
            'low_topic': '/cmd_vel_nav',
            'output_topic': '/cmd_vel'
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'motors_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for ZLAC8015D motor driver'
        ),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='Log level (debug|info|warn|error)'
        ),
        LogInfo(msg='Starting rover hardware: motors + twist mux + URDF'),
        robot_state_publisher_launch,
        twist_mux_node,
        wheels_driver_node,
    ])
