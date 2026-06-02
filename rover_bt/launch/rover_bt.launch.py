"""Launch the rover_bt behavior tree node and voice command node (hardware)."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_bt')

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
            description='Logging level (debug, info, warn, error)'),

        # ── rover_bt_node ──
        Node(
            package='rover_bt',
            executable='rover_bt_node',
            name='rover_bt_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'tree_xml': LaunchConfiguration('tree_xml')},
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
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
