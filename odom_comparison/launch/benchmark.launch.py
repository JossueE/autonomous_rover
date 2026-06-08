import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_output_root():
    candidates = [
        Path('/home/snorlix/colcon_ws/src/autonomous_rover/odom_comparison/images'),
        Path.cwd() / 'src' / 'autonomous_rover' / 'odom_comparison' / 'images',
        Path.cwd() / 'odom_comparison' / 'images',
    ]
    for candidate in candidates:
        if candidate.parent.exists():
            return str(candidate)
    return str(candidates[0])


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


def _include(package, launch_file, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def _launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot').perform(context)
    profile = _load_robot_profile(robot_name)
    log_level = LaunchConfiguration('log_level')
    output_root = LaunchConfiguration('output_root')
    rtabmap_mode = LaunchConfiguration('rtabmap_mode')

    wheels_separation = float(profile['wheels_separation'])
    wheel_radius = float(profile['wheel_radius'])
    driver_package = profile['driver_package']

    robot_state_publisher = _include(
        'robot_core',
        'robot_state_publisher.launch.py',
        {
            'wheel_separation': str(wheels_separation),
            'camera_x': str(profile['camera_x']),
            'camera_z': str(profile['camera_z']),
            'camera_pitch': str(profile.get('camera_pitch', 0.0)),
        },
    )

    if robot_name == 'zlac8015d':
        driver_arguments = [LaunchConfiguration('motors_port')]
    else:
        driver_arguments = [
            LaunchConfiguration('motor_left_port'),
            LaunchConfiguration('motor_right_port'),
        ]

    wheels_driver = Node(
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
            'wheel_radius': wheel_radius,
            'resolution_mode': True,
        }],
    )

    twist_mux = Node(
        package='rover_bringup',
        executable='twist_priority_mux.py',
        name='twist_priority_mux',
        output='screen',
        ros_arguments=['--log-level', log_level],
        parameters=[{
            'high_topic': '/cmd_vel_test',
            'person_topic': '/cmd_vel_unused_person',
            'low_topic': '/cmd_vel_unused_nav',
            'output_topic': '/cmd_vel',
            'timeout': 0.5,
            'rate_hz': 20.0,
        }],
    )

    wheel_odom = Node(
        package='odometry2',
        executable='odometry2',
        name='odometry2',
        output='screen',
        ros_arguments=['--log-level', log_level],
        parameters=[{
            'wheels_separation': wheels_separation,
            'wheel_radius': wheel_radius,
            'publish_tf': False,
        }],
    )

    rtabmap = _include(
        'rover_bringup',
        'k4a_rtabmap.launch.py',
        {'mode': rtabmap_mode, 'log_level': log_level},
    )

    recorder = Node(
        package='odom_comparison',
        executable='odom_compare_recorder',
        name='odom_compare_recorder',
        output='screen',
        parameters=[{
            'output_root': output_root,
            'auto_start': False,
            'wheel_topic': '/wheel/odom',
            'rtabmap_topic': '/rtabmap/odom',
            'rtabmap_info_lite_topic': '/rtabmap/odom_info_lite',
            'rtabmap_info_topic': '/rtabmap/odom_info',
        }],
    )

    return [
        LogInfo(msg='Starting independent odometry benchmark stack. Do not run Nav2/BT/teleop concurrently.'),
        robot_state_publisher,
        twist_mux,
        wheels_driver,
        wheel_odom,
        rtabmap,
        recorder,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot',
            default_value='zlac706',
            choices=['zlac706', 'zlac8015d'],
            description='Robot profile for hardware and odometry parameters.',
        ),
        DeclareLaunchArgument(
            'motor_left_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for left ZLAC706 motor driver.',
        ),
        DeclareLaunchArgument(
            'motor_right_port',
            default_value='/dev/ttyUSB1',
            description='Serial port for right ZLAC706 motor driver.',
        ),
        DeclareLaunchArgument(
            'motors_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for ZLAC8015D motor driver.',
        ),
        DeclareLaunchArgument(
            'rtabmap_mode',
            default_value='localization',
            choices=['localization', 'mapping', 'slam'],
            description='RTAB-Map mode used by rover_bringup/k4a_rtabmap.launch.py.',
        ),
        DeclareLaunchArgument(
            'output_root',
            default_value=_default_output_root(),
            description='Directory where recorder creates one folder per run.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='warn',
            description='Log level for launched nodes.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
