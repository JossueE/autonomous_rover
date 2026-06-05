"""Launcher general: hardware + Kinect/detector + visualización.

Lanza en un solo comando:
  1. rover_bringup.hardware_bringup  (driver de ruedas)
  2. person_tracker.run_all         (Kinect + nodo detector)
  3. image_view                      (visualización de detecciones)

Uso:
  ros2 launch person_tracker bringup.launch.py robot:=zlac706        # outdoor (default)
  ros2 launch person_tracker bringup.launch.py robot:=zlac706 mode:=indoor
  ros2 launch person_tracker bringup.launch.py robot:=zlac8015d motors_port:=/dev/WHEELS
  ros2 launch person_tracker bringup.launch.py mode:=indoor use_image_view:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='zlac706',
        choices=['zlac8015d', 'zlac706'],
        description='Perfil de robot usado por rover_bringup.hardware_bringup',
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='outdoor',
        description='Modo de navegación del detector: outdoor o indoor',
    )

    motors_port_arg = DeclareLaunchArgument(
        'motors_port',
        default_value='/dev/ttyUSB0',
        description='Puerto serial del driver de ruedas ZLAC8015D (ej. /dev/ttyUSB0, /dev/WHEELS)',
    )

    motor_left_port_arg = DeclareLaunchArgument(
        'motor_left_port',
        default_value='/dev/ttyUSB0',
        description='Puerto serial del driver izquierdo ZLAC706',
    )

    motor_right_port_arg = DeclareLaunchArgument(
        'motor_right_port',
        default_value='/dev/ttyUSB1',
        description='Puerto serial del driver derecho ZLAC706',
    )

    use_image_view_arg = DeclareLaunchArgument(
        'use_image_view',
        default_value='true',
        description='Si lanza la ventana de image_view con las detecciones anotadas',
    )

    rover_pkg_dir = get_package_share_directory('rover_bringup')
    detector_pkg_dir = get_package_share_directory('person_tracker')

    # 1. Hardware del rover (motores + state publisher)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rover_pkg_dir, 'launch', 'hardware_bringup.launch.py')
        ),
        launch_arguments={
            'robot': LaunchConfiguration('robot'),
            'motors_port': LaunchConfiguration('motors_port'),
            'motor_left_port': LaunchConfiguration('motor_left_port'),
            'motor_right_port': LaunchConfiguration('motor_right_port'),
        }.items(),
    )

    # 2. Cámara Kinect + detector (propaga el modo elegido)
    detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(detector_pkg_dir, 'launch', 'run_all.launch.py')
        ),
        launch_arguments={'mode': LaunchConfiguration('mode')}.items(),
    )

    # 3. Visualización del stream anotado
    image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='image_view',
        output='screen',
        remappings=[('image', '/person_tracker/detections_image')],
        condition=IfCondition(LaunchConfiguration('use_image_view')),
    )

    return LaunchDescription([
        robot_arg,
        mode_arg,
        motors_port_arg,
        motor_left_port_arg,
        motor_right_port_arg,
        use_image_view_arg,
        hardware_launch,
        detector_launch,
        image_view_node,
    ])
