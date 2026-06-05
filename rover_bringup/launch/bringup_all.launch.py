"""Top-level launcher for the real rover: hardware + localization + planner + NMPC + BT + RViz.

Brings up, in order:
  1. Robot hardware      (rover_bringup/hardware_bringup.launch.py)  — motors, twist mux, URDF
  2. Localization        (rover_bringup/k4a_rtabmap.launch.py)       — Azure Kinect + RTAB-Map -> /rtabmap/odom + map TF
  3. Path planner + RViz (path_planning_dynamic/planning.launch.py)  — navigate_to_goal action server, /sdv_trajectory, RViz
  4. NMPC controller     (nmpc_controller/sim_nmpc.launch.py)        — /sdv_trajectory -> /cmd_vel_safe
  5. Behaviour tree      (rover_bt/rover_bt.launch.py)               — the brain + voice node

Layers 3-5 are delayed a few seconds so localization/odom is alive first
(the BT's odom watchdog latches EMERGENCY if /rtabmap/odom is missing at boot;
it recovers on its own, but staggering avoids the noise).

The planner's RViz is forced to rover_bringup/rviz/nav.rviz so the planner
topics (route, /sdv_trajectory, occupancy grid, obstacles, footprint) are visible.

Usage:
    ros2 launch rover_bringup bringup_all.launch.py
    ros2 launch rover_bringup bringup_all.launch.py rtabmap_mode:=localization

Then drive it to a named lanelet (note the spaces after the colons):
    ros2 service call /rover_bt/send_command rover_bt/srv/SendCommand \
        "{source: 'gui', command: 'navigate', target: 'inicio'}"
Named lanelets in maps/test1.osm: inicio, banda, repiza, estacion, fin.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package, launch_file, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), 'launch', launch_file)
        ),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    pkg_bringup = get_package_share_directory('rover_bringup')
    nav_rviz = os.path.join(pkg_bringup, 'rviz', 'nav.rviz')

    rtabmap_mode = LaunchConfiguration('rtabmap_mode')
    log_level = LaunchConfiguration('log_level')
    robot = LaunchConfiguration('robot')

    # 1 + 2 — hardware and localization come up immediately.
    hardware = _include('rover_bringup', 'hardware_bringup.launch.py',
                        {'robot': robot, 'log_level': log_level})
    localization = _include(
        'rover_bringup', 'k4a_rtabmap.launch.py',
        {'mode': rtabmap_mode, 'log_level': log_level},
    )

    # 3 — planner + RViz (config forced to the real nav.rviz). Delayed so
    #     localization is settling before the planner builds the global route.
    planner = TimerAction(period=8.0, actions=[
        _include(
            'path_planning_dynamic', 'planning.launch.py',
            {'robot': robot, 'use_sim_time': 'False', 'rviz_config': nav_rviz, 'log_level': 'warn'},
        ),
    ])

    # 4 — NMPC velocity controller (/sdv_trajectory -> /cmd_vel_safe).
    nmpc = TimerAction(period=10.0, actions=[
        _include('nmpc_controller', 'sim_nmpc.launch.py', {'robot': robot, 'use_sim_time': 'False'}),
    ])

    # 5 — behaviour tree (the brain) + voice node, once everything else exists.
    bt = TimerAction(period=12.0, actions=[
        _include('rover_bt', 'rover_bt.launch.py', {'log_level': log_level}),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'rtabmap_mode', default_value='localization',
            choices=['localization', 'mapping', 'slam'],
            description='RTAB-Map mode: localization (existing map), mapping (fresh), slam (continue).'),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='Log level for all nodes (debug|info|warn|error).'),
        DeclareLaunchArgument(
            'robot', default_value='zlac706',
            choices=['zlac8015d', 'zlac706'],
            description='Robot profile used for hardware, planner, and controller parameters.'),

        LogInfo(msg='=== Rover full bring-up: hardware + localization + planner + NMPC + BT ==='),
        hardware,
        localization,
        planner,
        nmpc,
        bt,
    ])
