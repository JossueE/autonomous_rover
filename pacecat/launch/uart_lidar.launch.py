#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os


def generate_launch_description():
    share_dir = get_package_share_directory('pacecat')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'bluesea_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'uart_lidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')


    ROS_DISTRO=''
    ROS_DISTRO = os.getenv('ROS_DISTRO')
    print("Current ROS2 Version: ",ROS_DISTRO)
    if ROS_DISTRO[0] <= 'e':
        try:
            driver_node = LifecycleNode( node_name='bluesea_node', node_namespace='/', package='pacecat', node_executable='pacecat_node', output='screen', parameters=[parameter_file])
        except:
            pass
    else :
        try:
            driver_node = LifecycleNode( name='bluesea_node', namespace='/', package='pacecat', executable='pacecat_node', output='screen', emulate_tty=True, parameters=[parameter_file])
        except:
            pass

    crop_node = Node(
        package='pacecat',
        executable='pacecat_lidar_crop_node',
        name='pacecat_lidar_crop_node',
        output='screen',
        parameters=[
            {
                'input_topic': 'scan_raw',
                'output_topic': 'scan',
                'limit_angle': 2.094,
                'centered': True,
            }
        ]
    )

    return LaunchDescription([
        params_declare,
        driver_node,
        crop_node,
    ])
