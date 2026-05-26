import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    sensor_topic = LaunchConfiguration('sensor_topic')
    sensor_data_type = LaunchConfiguration('sensor_data_type')
    map_topic = LaunchConfiguration('map_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_rviz = LaunchConfiguration('rviz')

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'autostart': autostart,
                'use_sim_time': use_sim_time,
                'robot_base_frame': 'base_footprint',
                'robot_base_frame_id': 'base_footprint',
                'base_frame': 'base_footprint',
                'odom_topic': odom_topic,
                'cmd_vel_in_topic': 'cmd_vel_nav',
                'cmd_vel_out_topic': cmd_vel_topic,
                'topic': sensor_topic,
                'data_type': sensor_data_type,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    nav_remappings = remappings + [('map', map_topic)]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetParameter('use_sim_time', use_sim_time),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically activate Nav2 lifecycle nodes',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
            description='Base Nav2 params file',
        ),
        DeclareLaunchArgument(
            'sensor_topic',
            default_value='/scan',
            description='Obstacle topic for Nav2 costmaps',
        ),
        DeclareLaunchArgument(
            'sensor_data_type',
            default_value='LaserScan',
            description='Nav2 costmap sensor data type: LaserScan or PointCloud2',
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/rtabmap/map',
            description='Occupancy grid map topic produced by RTAB-Map',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/rtabmap/odom',
            description='Odometry topic produced by RTAB-Map',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel_safe',
            description='Velocity command topic sent to the rover safety layer',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Open the default Nav2 RViz view',
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params],
            remappings=nav_remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[configured_params],
            remappings=nav_remappings,
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params],
            remappings=nav_remappings,
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params],
            remappings=nav_remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=nav_remappings,
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params],
            remappings=nav_remappings,
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[configured_params],
            remappings=nav_remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2_nav2',
            output='screen',
            arguments=[
                '-d',
                os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'),
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
