from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lidar_model = LaunchConfiguration('lidar_model')
    base_color = LaunchConfiguration('base_color')
    camera_x = LaunchConfiguration('camera_x')
    camera_z = LaunchConfiguration('camera_z')
    camera_pitch = LaunchConfiguration('camera_pitch')

    xacro_file = PathJoinSubstitution([
        FindPackageShare('robot_core'),
        'urdf',
        'minibase_urdf.xacro',
    ])

    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file,
            ' LIDAR_MODEL:=',
            lidar_model,
            ' base_color:="',
            base_color,
            '"',
            ' x_offset:=',
            camera_x,
            ' z_offset:=',
            camera_z,
            ' pitch_angle:=',
            camera_pitch,
        ])
    }

    return LaunchDescription([
        DeclareLaunchArgument('lidar_model', default_value='pacecat'),
        DeclareLaunchArgument('base_color', default_value='0.45 0.48 0.50 1.0'),
        DeclareLaunchArgument('camera_x', default_value='0.15'),
        DeclareLaunchArgument('camera_z', default_value='0.43'),
        DeclareLaunchArgument('camera_pitch', default_value='-0.6'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen',
        ),
    ])
