from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eyes_gui',
            executable='robot_face_debug_ui',
            output='screen',
        ),
    ])
