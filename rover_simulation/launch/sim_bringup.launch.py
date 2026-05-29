"""Gazebo Harmonic simulation bringup for the minibase rover.

Uses the local depot world (world name: world_demo).
All optional components are guarded by IfCondition arguments so the launch
can be used for incremental testing:

  # Minimum (Gazebo + robot + sensors + odometry):
  ros2 launch rover_simulation sim_bringup.launch.py

  # With RTAB-Map mapping:
  ros2 launch rover_simulation sim_bringup.launch.py use_rtabmap:=true

  # Full stack (planning + BT):
  ros2 launch rover_simulation sim_bringup.launch.py use_rtabmap:=true use_planning:=true use_voice_bt:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_sim     = get_package_share_directory("rover_simulation")
    pkg_core    = get_package_share_directory("robot_core")
    pkg_planning = get_package_share_directory("path_planning_dynamic")

    # ── Launch arguments ────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Pass use_sim_time:=true to all nodes"),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg_sim, "worlds", "depot.sdf"),
            description="Path to Gazebo world SDF"),
        DeclareLaunchArgument(
            "spawn_x", default_value="0.0"),
        DeclareLaunchArgument(
            "spawn_y", default_value="0.0"),
        DeclareLaunchArgument(
            "spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "use_rtabmap", default_value="false",
            description="Launch RTAB-Map SLAM node"),
        DeclareLaunchArgument(
            "use_planning", default_value="false",
            description="Launch Lanelet2 path planner + NMPC controller"),
        DeclareLaunchArgument(
            "use_voice_bt", default_value="false",
            description="Launch voice-commanded behavior tree"),
        DeclareLaunchArgument(
            "use_rviz", default_value="true",
            description="Launch RViz2"),
        # map -> odom placement (sim localization stand-in; see static_map_odom_tf).
        # Defaults put the robot at the start of the "inicio" lanelet of
        # test_simulation.osm, facing along it, so the planner has a valid pose
        # on the route. Values are map-frame coords from the lanelet local_x/local_y.
        DeclareLaunchArgument(
            "map_odom_x", default_value="0.14",
            description="map->odom x: places the robot on the start lanelet (map frame)"),
        DeclareLaunchArgument(
            "map_odom_y", default_value="1.67",
            description="map->odom y: places the robot on the start lanelet (map frame)"),
        DeclareLaunchArgument(
            "map_odom_yaw", default_value="-1.5708",
            description="map->odom yaw: aligns robot heading with the start lanelet"),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")
    world        = LaunchConfiguration("world")
    spawn_x      = LaunchConfiguration("spawn_x")
    spawn_y      = LaunchConfiguration("spawn_y")
    spawn_yaw    = LaunchConfiguration("spawn_yaw")
    use_rtabmap  = LaunchConfiguration("use_rtabmap")
    use_planning = LaunchConfiguration("use_planning")
    use_voice_bt = LaunchConfiguration("use_voice_bt")
    use_rviz     = LaunchConfiguration("use_rviz")
    map_odom_x   = LaunchConfiguration("map_odom_x")
    map_odom_y   = LaunchConfiguration("map_odom_y")
    map_odom_yaw = LaunchConfiguration("map_odom_yaw")

    # ── 1. Robot State Publisher ─────────────────────────────────────────────
    # Uses minibase_sim.xacro which includes minibase_urdf.xacro with sim:=true.
    sim_xacro = os.path.join(pkg_core, "urdf", "minibase_sim.xacro")
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command([
                    FindExecutable(name="xacro"), " ", sim_xacro,
                    " sim:=true",
                    " LIDAR_MODEL:=pacecat",
                ]),
                value_type=str,
            ),
            "use_sim_time": use_sim_time,
        }],
    )

    # ── 2. Gazebo Harmonic ───────────────────────────────────────────────────
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": [world, " -r"],
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── 3. Spawn robot ───────────────────────────────────────────────────────
    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_rover",
            output="screen",
            arguments=[
                "-world", "world_demo",
                "-topic", "robot_description",
                "-name",  "rover",
                "-x",     spawn_x,
                "-y",     spawn_y,
                "-z",     "0.12",
                "-Y",     spawn_yaw,
            ],
        )],
    )

    # ── 4. ros_gz_bridge ────────────────────────────────────────────────────
    bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{
            "config_file": os.path.join(pkg_sim, "config", "ros_gz_bridge.yaml"),
            "use_sim_time": use_sim_time,
        }],
    )

    # ── 5. IMU filter (Madgwick) ─────────────────────────────────────────────
    # Processes /imu/data_raw (WitMotion sim) → /imu/data for the EKF.
    imu_filter_node = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        output="screen",
        parameters=[{
            "use_mag":     False,
            "world_frame": "enu",
            "publish_tf":  False,
            "use_sim_time": use_sim_time,
        }],
    )

    # ── 6. EKF node ──────────────────────────────────────────────────────────
    # Fuses /wheel/odom + /imu/data → /odom and TF odom→base_footprint.
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        remappings=[("odometry/filtered", "odom")],
        parameters=[
            os.path.join(pkg_sim, "config", "ekf_sim.yaml"),
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── 7. RTAB-Map (optional) ───────────────────────────────────────────────
    # Uses simulated sensor topics instead of real Kinect topics.
    # Kinect IMU (/depth_camera/imu) is used rather than WitMotion for SLAM,
    # matching the hardware configuration where RTAB-Map uses /k4a/imu_filtered.
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("rtabmap_launch"),
                "launch",
                "rtabmap.launch.py",
            ])
        ]),
        launch_arguments={
            "rgb_topic":               "/depth_camera/image",
            "depth_topic":             "/depth_camera/depth_image",
            "camera_info_topic":       "/depth_camera/camera_info",
            "imu_topic":               "/depth_camera/imu",
            "wait_imu_to_init":        "true",
            "frame_id":                "base_footprint",
            "approx_sync":             "true",
            "approx_sync_max_interval": "0.05",
            "wait_for_transform":      "0.3",
            "queue_size":              "20",
            "use_sim_time":            use_sim_time,
            "rviz":                    "false",
            "rtabmap_args": (
                "--delete_db_on_start --Reg/Force3DoF true "
                "--Grid/FromDepth true --Grid/3D false "
                "--Grid/RangeMax 4.5 --Grid/MaxGroundHeight 0.10 "
                "--Grid/MaxObstacleHeight 1.20 --Grid/CellSize 0.05"
            ),
        }.items(),
        condition=IfCondition(use_rtabmap),
    )

    # ── 7.5 Point cloud (native ROS generation) ─────────────────────────────
    # Replaces the Gazebo-bridged cloud, which is misaligned because Gazebo
    # always generates XYZ along its own X-forward axis while stamping the
    # message with the optical frame. point_cloud_xyz reads the depth image
    # (already correct thanks to the optical-frame rotation in the URDF) and
    # publishes to /depth_camera/points so all downstream nodes are unaffected.
    point_cloud_xyz_node = Node(
        package="rtabmap_util",
        executable="point_cloud_xyz",
        name="point_cloud_xyz_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("depth/image",       "/depth_camera/depth_image"),
            ("depth/camera_info", "/depth_camera/camera_info"),
            ("cloud",             "/depth_camera/points"),
        ],
    )

    # ── 8. Path planning nodes (optional) ────────────────────────────────────
    # Launched individually (not via planning.launch.py) so pointcloud_roi_node
    # can be given the simulation pointcloud topic (/depth_camera/points).
    planning_params = os.path.join(pkg_planning, "config", "params.yaml")

    # The lanelet2 loader (lanelet::load) does not understand "package://" URIs,
    # so the shared params.yaml map_path fails to load in any environment.
    # Override with a resolved absolute path to the simulation map for sim runs.
    sim_map_path = os.path.join(
        get_package_share_directory("rover_bringup"),
        "maps", "test_simulation.osm",
    )

    path_planner_node = Node(
        package="path_planning_dynamic",
        executable="path_planning_node",
        name="path_planning_node",
        output="screen",
        parameters=[
            planning_params,
            {
                "use_sim_time": use_sim_time,
                "map_path": sim_map_path,
                # The shared params.yaml names ("home"/"station1") do not exist in
                # test_simulation.osm. Its named lanelets are inicio/cocina/fin, so
                # override here (sim-only) — otherwise resolveLaneletName fails and
                # the planner falls back to nonexistent IDs and builds no route.
                "start_lanelet_name": "inicio",
                "end_lanelet_name": "cocina",
            },
        ],
        condition=IfCondition(use_planning),
    )

    # Static map -> odom transform (sim localization stand-in).
    # The sim EKF runs with world_frame=odom, so it only publishes odom ->
    # base_footprint; nothing publishes the "map" frame. The path planner does
    # lookupTransform("map", base_footprint) every cycle, so without this the
    # lookup throws, the robot pose is never valid, and no trajectory is ever
    # produced. This static transform places the odom origin (robot spawn) onto
    # the start lanelet of the map so the planner has a pose on the route.
    # Disabled when use_rtabmap:=true, since RTAB-Map (SLAM) then publishes a
    # real map -> odom and a static one would conflict.
    static_map_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_odom_tf",
        output="screen",
        arguments=[
            "--x", map_odom_x, "--y", map_odom_y, "--z", "0.0",
            "--yaw", map_odom_yaw, "--pitch", "0.0", "--roll", "0.0",
            "--frame-id", "map", "--child-frame-id", "odom",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=UnlessCondition(use_rtabmap),
    )

    pointcloud_clustering_node = Node(
        package="path_planning_dynamic",
        executable="pointcloud_clustering_node",
        name="pointcloud_clustering_node",
        output="screen",
        parameters=[planning_params, {"use_sim_time": use_sim_time}],
        condition=IfCondition(use_planning),
    )

    # pointcloud_roi_node with sim topic override for pointcloud input.
    # frame_id stays "depth_camera_link" (exists in sim URDF under sim:=true).
    pointcloud_roi_node = Node(
        package="path_planning_dynamic",
        executable="pointcloud_roi_node",
        name="pointcloud_roi_node",
        output="screen",
        parameters=[
            planning_params,
            {
                "pointcloud_topic": "/depth_camera/points",
                "use_sim_time": use_sim_time,
            },
        ],
        condition=IfCondition(use_planning),
    )

    # ── 9. NMPC controller (optional, requires planning) ─────────────────────
    nmpc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nmpc_controller"),
                "launch",
                "sim_nmpc.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time":   use_sim_time,
            "cmd_vel_topic":  "/cmd_vel_safe",
        }.items(),
        condition=IfCondition(use_planning),
    )

    # ── 10. Rover BT (optional) ───────────────────────────────────────────────
    # Launches the new C++ Behavior Tree node alongside the Python ASR node
    # by invoking the new rover_bt_sim.launch.py script.
    rover_bt_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rover_bt"),
                "launch",
                "rover_bt_sim.launch.py",
            )
        ),
        condition=IfCondition(use_voice_bt),
    )

    # ── 11. RViz2 (optional) ─────────────────────────────────────────────────
    rviz_config = os.path.join(
        get_package_share_directory("rover_bringup"), "rviz", "sim.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        args + [
            LogInfo(msg="=== Rover Gazebo Harmonic Simulation Bringup ==="),
            rsp_node,
            gz_sim_launch,
            spawn_robot,
            bridge_node,
            imu_filter_node,
            ekf_node,
            rtabmap_launch,
            point_cloud_xyz_node,
            static_map_odom_tf,
            path_planner_node,
            pointcloud_clustering_node,
            pointcloud_roi_node,
            nmpc_launch,
            rover_bt_launch,
            rviz_node,
        ]
    )
