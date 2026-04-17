import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declaración de argumentos de lanzamiento (visibles desde la terminal)
    motors_port_arg = DeclareLaunchArgument(
        'motors_port',
        default_value='/dev/ttyUSB0',
        description='Puerto serial para el driver de motores ZLAC8015D'
    )

    imu_port_arg = DeclareLaunchArgument(
        'imu_port',
        default_value='/dev/ttyUSB1',
        description='Puerto serial para el sensor IMU WitMotion'
    )

    # 2. Captura de los valores de los argumentos
    motors_port = LaunchConfiguration('motors_port')
    imu_port = LaunchConfiguration('imu_port')

    # Ruta al archivo de configuración EKF (asegúrate de que este archivo exista)
    # Recomiendo ponerlo en rover_bringup/config/ekf.yaml
    pkg_bringup = get_package_share_directory('rover_bringup')
    ekf_config_path = os.path.join(pkg_bringup, 'config', 'ekf.yaml')

    # Node: ZLAC8015D Motor Driver
    wheels_driver_node = Node(
        package='zlac8015d_driver2_cpp',
        executable='wheels_driver',
        name='wheels_driver',
        output='screen',
        arguments=[motors_port], # Pasa el puerto como argumento de línea de comandos
        parameters=[{
            'unlock_driver': True,
            'accel_time_ms': 500,
            'decel_time_ms': 500
        }]
    )

    # Node: Odometry (Encoders)
    odometry_node = Node(
        package='odometry2',
        executable='odometry2',
        name='odometry2_node',
        output='screen',
        parameters=[{
            'publish_tf': False, # IMPORTANTE: False para dejar que el EKF maneje el TF
            'wheels_separation': 0.4,
            'wheel_radius': 0.1
        }]
    )

    # Node: WitMotion IMU
    imu_node = Node(
        package='odometry2',
        executable='imu',
        name='imu_node',
        output='screen',
        arguments=[imu_port], # Pasa el puerto como argumento de línea de comandos
        parameters=[{
            'baudrate': 115200,
            'frame_id': 'imu_link'
        }]
    )

    # Node: EKF Fusion (Robot Localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    return LaunchDescription([
        LogInfo(msg='Iniciando Hardware del Rover con Fusión EKF...'),
        motors_port_arg,
        imu_port_arg,
        wheels_driver_node,
        odometry_node,
        imu_node,
        ekf_node
    ])