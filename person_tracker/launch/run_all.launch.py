import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detector_pkg_dir = get_package_share_directory('person_tracker')

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='indoor',
        description='Modo de navegación: outdoor (rápido, exterior) o indoor (suave, laboratorio)',
    )

    # 1. Kinect a 720P/30fps: menos ancho de banda USB y más frames para YOLO+ReID.
    #    720P (1280x720) coincide con los intrínsecos usados para camera_hfov_deg=92.6.
    k4a_launch = Node(
        package='azure_kinect_ros2_driver',
        executable='azure_kinect_node',
        name='k4a_ros2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'depth_mode': 'NFOV_2X2BINNED',
            'color_resolution': '720P',
            'fps': 30,
        }],
    )

    # 2. Detector con el modo pasado como argumento
    detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detector_pkg_dir, 'launch', 'person_tracker.launch.py')
        ),
        launch_arguments={'mode': LaunchConfiguration('mode')}.items(),
    )

    return LaunchDescription([
        mode_arg,
        k4a_launch,
        detector_launch,
    ])
