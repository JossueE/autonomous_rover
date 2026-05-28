"""Launch file for the voice-commanded behavior tree."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("voice_bt")
    assets = os.path.join(pkg_share, "voice_assets")

    args = [
        DeclareLaunchArgument(
            "vosk_model_es",
            default_value=os.path.join(assets, "model_es"),
            description="Path to the Spanish Vosk model directory"),
        DeclareLaunchArgument(
            "vosk_model_en",
            default_value=os.path.join(assets, "model_en"),
            description="Path to the English Vosk model directory"),
        DeclareLaunchArgument(
            "piper_bin",
            default_value=os.path.join(assets, "piper", "piper"),
            description="Piper TTS binary"),
        DeclareLaunchArgument(
            "piper_model",
            default_value=os.path.join(assets, "es_MX-claude-high.onnx"),
            description="Piper voice model (.onnx)"),
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=os.path.join(pkg_share, "config", "waypoints.yaml"),
            description="YAML file with named waypoints"),
        DeclareLaunchArgument(
            "cmd_vel_topic", default_value="/cmd_vel_safe"),
        DeclareLaunchArgument(
            "amcl_pose_topic", default_value="/amcl_robot_pose"),
        DeclareLaunchArgument(
            "odom_topic", default_value="/odom"),
        DeclareLaunchArgument(
            "bt_tick_rate", default_value="10.0"),
    ]

    voice_bt_node = Node(
        package="voice_bt",
        executable="voice_bt_node",
        name="voice_bt_node",
        output="screen",
        parameters=[{
            "vosk_model_es": LaunchConfiguration("vosk_model_es"),
            "vosk_model_en": LaunchConfiguration("vosk_model_en"),
            "piper_bin": LaunchConfiguration("piper_bin"),
            "piper_model": LaunchConfiguration("piper_model"),
            "waypoints_file": LaunchConfiguration("waypoints_file"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "amcl_pose_topic": LaunchConfiguration("amcl_pose_topic"),
            "odom_topic": LaunchConfiguration("odom_topic"),
            "bt_tick_rate": LaunchConfiguration("bt_tick_rate"),
        }],
    )

    return LaunchDescription(args + [voice_bt_node])
