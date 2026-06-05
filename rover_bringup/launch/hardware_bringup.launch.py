import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_robot_profile(robot_name):
    profiles_path = os.path.join(
        get_package_share_directory('rover_bringup'),
        'config',
        'robot_profiles.yaml',
    )
    with open(profiles_path, 'r') as profiles_file:
        profiles = yaml.safe_load(profiles_file) or {}

    robots = profiles.get('robots', {})
    if robot_name not in robots:
        raise RuntimeError(f"Unknown robot profile '{robot_name}'. Expected one of: {', '.join(sorted(robots))}")
    return robots[robot_name]


def _launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot').perform(context)
    log_level = LaunchConfiguration('log_level')
    profile = _load_robot_profile(robot_name)
    driver_package = profile['driver_package']
    wheels_separation = float(profile['wheels_separation'])
    pkg_robot_core = get_package_share_directory('robot_core')

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_core, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'wheel_separation': str(wheels_separation),
            'camera_x': str(profile['camera_x']),
            'camera_z': str(profile['camera_z']),
            'camera_pitch': str(profile.get('camera_pitch', 0.0)),
        }.items(),
    )

    if robot_name == 'zlac8015d':
        driver_arguments = [LaunchConfiguration('motors_port')]
    else:
        driver_arguments = [
            LaunchConfiguration('motor_left_port'),
            LaunchConfiguration('motor_right_port'),
        ]

    wheels_driver_node = Node(
        package=driver_package,
        executable='wheels_driver',
        name='wheels_driver',
        output='screen',
        arguments=driver_arguments,
        ros_arguments=['--log-level', log_level],
        parameters=[{
            'unlock_driver': True,
            'accel_time_ms': 500,
            'decel_time_ms': 500,
            'wheels_separation': wheels_separation,
            'resolution_mode': True,
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

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'autorepeat_rate': 20.0,
        }],
        ros_arguments=['--log-level', log_level],
    )

    teleop_joycon_node = Node(
        package='teleop',
        executable='teleop_joycon',
        name='teleop_joycon',
        output='screen',
        ros_arguments=['--log-level', log_level],
    )

    return [
        LogInfo(msg=f'Starting rover hardware profile: {robot_name}'),
        robot_state_publisher_launch,
        joy_node,
        teleop_joycon_node,
        twist_mux_node,
        wheels_driver_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='zlac706',
            choices=['zlac8015d', 'zlac706'],
            description='Robot hardware profile'
        ),
        DeclareLaunchArgument(
            'motor_left_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for left ZLAC706 motor driver'
        ),
        DeclareLaunchArgument(
            'motor_right_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for right ZLAC706 motor driver'
        ),
        DeclareLaunchArgument(
            'motors_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the ZLAC8015D motor driver'
        ),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='Log level (debug|info|warn|error)'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
