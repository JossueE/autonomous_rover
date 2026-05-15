import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("nmpc_controller")
    default_params = os.path.join(package_share, "config", "sim_nmpc.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    costmap_topic = LaunchConfiguration("costmap_topic")
    path_topic = LaunchConfiguration("path_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
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
                default_value="/cmd_vel",
                description="Velocity command output topic",
            ),
            DeclareLaunchArgument(
                "costmap_topic",
                default_value="/global_planner_occupancy_grid",
                description="Occupancy grid topic consumed by NMPC",
            ),
            DeclareLaunchArgument(
                "path_topic",
                default_value="/sdv_trajectory",
                description="Path topic consumed by NMPC",
            ),
            Node(
                package="nmpc_controller",
                executable="nmpc_controller_node",
                output="screen",
                parameters=[
                    params_file,
                    {
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
    )
