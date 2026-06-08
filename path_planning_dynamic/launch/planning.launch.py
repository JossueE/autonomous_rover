import os
from pathlib import Path

import launch
import launch_ros
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def load_ros_parameters(config_path: str, node_name: str) -> dict:
    with open(config_path, 'r') as config_file:
        config = yaml.safe_load(config_file) or {}

    node_config = config.get(node_name) or config.get('/**') or {}
    return node_config.get('ros__parameters', node_config)


def resolve_map_path(raw_path: str, default_pkg_path: str) -> str:
    expanded_path = Path(os.path.expandvars(os.path.expanduser(raw_path)))
    if raw_path.startswith('package://'):
        package_name, _, relative_path = raw_path.removeprefix('package://').partition('/')
        if not package_name or not relative_path:
            raise ValueError(f'Invalid package URI: {raw_path}')
        return str((Path(get_package_share_directory(package_name)) / relative_path).resolve())

    if expanded_path.is_absolute():
        return str(expanded_path)

    default_pkg = Path(default_pkg_path).resolve()
    workspace_root = default_pkg.parents[3]
    install_root = default_pkg.parents[2]

    candidates = [
        default_pkg / expanded_path,
        workspace_root / expanded_path,
        install_root / expanded_path,
    ]

    for candidate in candidates:
        if candidate.exists():
            return str(candidate.resolve())

    return str((default_pkg / expanded_path).resolve())


def load_robot_profile(robot_name: str) -> dict:
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


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot').perform(context)
    profile = load_robot_profile(robot_name)
    package_path = get_package_share_directory('path_planning_dynamic')
    paramsConfig = os.path.join(package_path, 'config', 'params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    map_path = resolve_map_path(
        profile['map_path'],
        package_path,
    )
    roi_max_y = float(profile['pointcloud_roi']['roi_max_y_'])
    rviz_config_arg = LaunchConfiguration('rviz_config')


    publisher_node_planner = launch_ros.actions.Node(
        package='path_planning_dynamic',
        executable='path_planning_node',
        name='path_planning_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[paramsConfig, {'map_path': map_path, 'use_sim_time': use_sim_time}],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    pointcloud_clustering = launch_ros.actions.Node(
        package='path_planning_dynamic',
        executable='pointcloud_clustering_node',
        name='pointcloud_clustering_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[paramsConfig, {'use_sim_time': use_sim_time}],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    pointcloud_roi = launch_ros.actions.Node(
        package='path_planning_dynamic',
        executable='pointcloud_roi_node',
        name='pointcloud_roi_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[paramsConfig, {'roi_max_y_': roi_max_y, 'use_sim_time': use_sim_time}],
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': "{message}"}
    )

    rviz = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_arg],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return [
        pointcloud_roi,
        pointcloud_clustering,
        publisher_node_planner,
        rviz,
    ]


def generate_launch_description():
    rviz_config_default = os.path.join(
        get_package_share_directory('rover_bringup'), 'rviz', 'nav.rviz'
    )

    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    rviz_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_default,
        description='Path to rviz config file'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='warn',
        description='Log level for path planner nodes (debug|info|warn|error).'
    )

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='zlac706',
        choices=['zlac8015d', 'zlac706'],
        description='Robot profile used for map and pointcloud ROI parameters'
    )
    
    return launch.LaunchDescription([
        robot_arg,
        simu_time,
        rviz_arg,
        log_level_arg,
        OpaqueFunction(function=launch_setup),
    ])
