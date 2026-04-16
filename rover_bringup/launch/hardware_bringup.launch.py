import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    """
    Generates a ROS 2 launch description to initialize all hardware components 
    of the differential drive rover.
    """
    
    # Configuration constants for USB ports
    # Consider using udev rules in the future for static symlinks (e.g., /dev/rover_motors)
    MOTORS_PORT = '/dev/ttyUSB0'
    IMU_PORT = '/dev/ttyUSB1'

    # Node: ZLAC8015D Motor Driver
    wheels_driver_node = Node(
        package='zlac8015d_driver2_cpp',
        executable='wheels_driver',
        name='wheels_driver',
        output='screen',
        arguments=[MOTORS_PORT],
        parameters=[{
            'unlock_driver': True,
            'time_disabled_driver_s': 3.0,
            'accel_time_ms': 500,  # Soft start for real hardware
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
            'publish_tf': True,
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
        arguments=[IMU_PORT],
        parameters=[{
            'baudrate': 9600,
            'frame_id': 'imu_link'
        }]
    )

    return LaunchDescription([
        LogInfo(msg='Starting Rover Hardware Bringup...'),
        wheels_driver_node,
        odometry_node,
        imu_node
    ])