"""Launch the rover_bt behavior tree node and voice command node for simulation.

Overrides sensor topics to use Gazebo-simulated equivalents.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_bt')
    pkg_voice_bt = get_package_share_directory('voice_bt')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'rover_bt_params.yaml'),
            description='Path to the rover_bt parameter file'),

        DeclareLaunchArgument(
            'tree_xml',
            default_value=os.path.join(pkg_dir, 'trees', 'rover_bt_main.xml'),
            description='Path to the BT XML tree file'),

        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the /clock topic for time (required in simulation)'),

        DeclareLaunchArgument(
            'dynamic_waypoints_file',
            default_value=os.path.join(pkg_dir, 'config', 'waypoints_sim.yaml'),
            description='Named locations (map frame) the BT can navigate to. '
                        'Defaults to the sim waypoints matched to test_simulation.osm.'),

        # ── rover_bt_node (sim overrides) ──
        Node(
            package='rover_bt',
            executable='rover_bt_node',
            name='rover_bt_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'tree_xml': LaunchConfiguration('tree_xml')},
                # Sim uses the Gazebo /clock; without this every sensor stamp
                # (sim time) is compared against wall-clock now(), so the
                # watchdogs see ~1.7e9 s of staleness and the odom critical
                # gate latches EMERGENCY forever.
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                # Sim locations the BT can navigate to. The shared params file
                # leaves dynamic_waypoints_file empty (registry would be empty,
                # so every "navigate <place>" fails as unknown). Point it at the
                # sim waypoints whose coords/lanelets match test_simulation.osm.
                {'dynamic_waypoints_file':
                    LaunchConfiguration('dynamic_waypoints_file')},
                # Simulation topic overrides
                {'pointcloud_topic': '/depth_camera/points'},
                {'rtabmap_rgb_topic': '/depth_camera/image'},
                {'rtabmap_depth_topic': '/depth_camera/depth_image'},
                {'rtabmap_camera_info_topic': '/depth_camera/camera_info'},
                {'rtabmap_scan_cloud_topic': '/depth_camera/points'},
                {'rtabmap_imu_topic': '/depth_camera/imu'},
            ],
            arguments=['--ros-args', '--log-level',
                       LaunchConfiguration('log_level')],
        ),

        # ── voice_command_node (ASR) ──
        Node(
            package='rover_bt',
            executable='voice_command_node.py',
            name='voice_command_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'vosk_model_es': os.path.join(pkg_voice_bt, 'voice_assets', 'model_es'),
                'vosk_model_en': os.path.join(pkg_voice_bt, 'voice_assets', 'model_en'),
                'waypoints_file': os.path.join(pkg_dir, 'config', 'waypoints.yaml'),
            }],
        ),
    ])
