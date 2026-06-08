import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_robot_profile(robot_name):
    profiles_path = os.path.join(
        get_package_share_directory("rover_bringup"),
        "config",
        "robot_profiles.yaml",
    )
    with open(profiles_path, "r") as profiles_file:
        profiles = yaml.safe_load(profiles_file) or {}

    robots = profiles.get("robots", {})
    if robot_name not in robots:
        raise RuntimeError(f"Unknown robot profile '{robot_name}'. Expected one of: {', '.join(sorted(robots))}")
    return robots[robot_name]


def _launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot").perform(context)
    profile = _load_robot_profile(robot_name)

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    costmap_topic = LaunchConfiguration("costmap_topic")
    path_topic = LaunchConfiguration("path_topic")

    return [
        Node(
            package="nmpc_controller",
            executable="nmpc_controller_node",
            output="screen",
            parameters=[
                params_file,
                {
                    "L": float(profile["nmpc"]["L"]),
                    "v_max": float(profile["nmpc"]["v_max"]),
                    "use_sim_time": use_sim_time,
                    "map_frame": map_frame,
                    "base_frame": base_frame,
                    "cmd_vel_topic": cmd_vel_topic,
                    "costmap_topic": costmap_topic,
                    "path_topic": path_topic,
                },
            ],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("nmpc_controller")
    default_params = os.path.join(package_share, "config", "sim_nmpc.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot",
                default_value="zlac706",
                choices=["zlac8015d", "zlac706"],
                description="Robot profile used for controller geometry",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False"   ,
                description="Use Gazebo simulation clock",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Parameter file for NMPC controller",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Control frame used for TF lookup and references",
            ),
            DeclareLaunchArgument(
                "base_frame",
                default_value="base_footprint",
                description="Robot base frame used for TF lookup",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel_nav",
                description="Velocity command output topic",
            ),
            DeclareLaunchArgument(
                "costmap_topic",
                default_value="/occupancy_grid_obstacles",
                description="Occupancy grid topic consumed by NMPC",
            ),
            DeclareLaunchArgument(
                "path_topic",
                default_value="/sdv_trajectory",
                description="Path topic consumed by NMPC",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
