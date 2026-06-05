"""Launch the rover_bt behavior tree node and voice command node (hardware)."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_bt')
    person_tracker_dir = get_package_share_directory('person_tracker')

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

        DeclareLaunchArgument(
            'use_person_tracker',
            default_value='true',
            description='Launch the person_tracker follower (gated: idle until the '
                        'BT enters PERSON_TRACK). Needs the Azure Kinect running.'),

        DeclareLaunchArgument(
            'person_tracker_mode',
            default_value='outdoor',
            description='Initial person_tracker tuning profile: outdoor or indoor '
                        '(switchable live via the person_track command target).'),

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

        # ── person_tracker (follow-me) ──
        # Launched gated (start_enabled:=false): the node sits idle — no YOLO/OSNet
        # inference, no cmd_vel — until rover_bt enters PERSON_TRACK and asserts
        # /person_tracker/enable. It publishes to /cmd_vel_person, which the twist
        # mux routes below /cmd_vel_safe. The Azure Kinect must be running already.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(person_tracker_dir, 'launch', 'person_tracker.launch.py')
            ),
            launch_arguments={
                'mode': LaunchConfiguration('person_tracker_mode'),
                'start_enabled': 'false',
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_person_tracker')),
        ),
    ])
