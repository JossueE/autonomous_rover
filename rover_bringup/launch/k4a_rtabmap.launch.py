import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    log_level = LaunchConfiguration('log_level')
    rtabmap_args = LaunchConfiguration('rtabmap_args')

    rtabmap_localization = PythonExpression([
        "'true' if '", mode, "' == 'localization' else 'false'"
    ])
    rtabmap_rviz = PythonExpression([
        "'true' if '", mode, "' == 'mapping' else 'false'"
    ])
    rtabmap_mode_args = [
        PythonExpression(["'--delete_db_on_start ' if '", mode, "' == 'mapping' else ''"]),
        rtabmap_args,
    ]

    # Kinect driver is inherently verbose (30 fps + IMU); send to log file only.
    k4a_node = Node(
        package='azure_kinect_ros2_driver',
        executable='azure_kinect_node',
        name='k4a_ros2_node',
        output='log',
        emulate_tty=True,
        parameters=[{
            'point_cloud': True,
            'rgb_point_cloud': False,
            'color_resolution': '1536P',
            'fps': 30,
        }],
    )

    # IMU filter runs at IMU rate; log file only.
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        output='log',
        parameters=[{
            'use_mag': False,
            'world_frame': 'enu',
            'publish_tf': False,
        }],
        remappings=[
            ('imu/data_raw', '/k4a/imu'),
            ('imu/data', '/k4a/imu_filtered'),
        ],
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py',
            )
        ),
        launch_arguments={
            'localization': rtabmap_localization,
            'visual_odometry': 'true',
            'icp_odometry': 'true',
            'odom_topic': 'odom',
            'publish_tf_odom': 'true',
            'rtabmap_args': rtabmap_mode_args,
            'rgb_topic': '/k4a/rgb/image_raw',
            'depth_topic': '/k4a/depth_to_rgb/image_raw',
            'camera_info_topic': '/k4a/rgb/camera_info',
            'scan_cloud_topic': '/k4a/points2',
            'subscribe_scan_cloud': 'true',
            'imu_topic': '/k4a/imu_filtered',
            'wait_imu_to_init': 'true',
            'frame_id': 'base_footprint',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.02',
            'wait_for_transform': '2.0',
            'queue_size': '20',
            'qos': '2',
            'rviz': rtabmap_rviz,
            'log_level': log_level,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='slam',
            choices=['localization', 'mapping', 'slam'],
            description='localization: use existing map, mapping: delete DB and map from zero, slam: continue/update existing map',
        ),
        DeclareLaunchArgument(
            'log_level', default_value='warn',
            description='Log level (debug|info|warn|error)',
        ),
        DeclareLaunchArgument(
            'rtabmap_args',
            default_value=(
                '--Reg/Force3DoF true '
                '--Reg/Strategy 2 '
                '--Grid/FromDepth false '
                '--Grid/3D false '
                '--Grid/RangeMax 5.0 '
                '--Grid/MaxGroundHeight 0.10 '
                '--Grid/MaxObstacleHeight 1.20 '
                '--Grid/CellSize 0.05'
            ),
        ),
        LogInfo(msg='Starting Azure Kinect + Madgwick IMU filter + RTAB-Map'),
        k4a_node,
        imu_filter_node,
        rtabmap_launch,
    ])
