# rover_simulation

Gazebo Harmonic simulation bringup for the minibase differential-drive rover.

This package installs simulation launch files, bridge configuration, EKF configuration, and Gazebo worlds.

## Main Commands

Minimum simulation:

```bash
ros2 launch rover_simulation sim_bringup.launch.py
```

Simulation with RTAB-Map:

```bash
ros2 launch rover_simulation sim_bringup.launch.py use_rtabmap:=true
```

Full simulation with planning and behavior tree:

```bash
ros2 launch rover_simulation sim_bringup.launch.py use_rtabmap:=true use_planning:=true use_bt:=true
```

Disable RViz:

```bash
ros2 launch rover_simulation sim_bringup.launch.py use_rviz:=false
```

## Launch Arguments

| Argument | Default | Purpose |
| --- | --- | --- |
| `use_sim_time` | `true` | Pass simulation time to launched nodes. |
| `world` | `worlds/depot.sdf` | Gazebo world SDF path. |
| `spawn_x` | `0.0` | Robot spawn X. |
| `spawn_y` | `0.0` | Robot spawn Y. |
| `spawn_yaw` | `0.0` | Robot spawn yaw. |
| `use_rtabmap` | `false` | Launch RTAB-Map SLAM. |
| `use_planning` | `false` | Launch path planner, point cloud obstacle nodes, and NMPC. |
| `use_bt` | `false` | Launch `rover_bt/rover_bt_sim.launch.py`. |
| `use_rviz` | `true` | Launch RViz with `rover_bringup/rviz/sim.rviz`. |
| `map_odom_x` | `0.14` | Static `map -> odom` X when RTAB-Map is disabled. |
| `map_odom_y` | `1.67` | Static `map -> odom` Y when RTAB-Map is disabled. |
| `map_odom_yaw` | `-1.5708` | Static `map -> odom` yaw when RTAB-Map is disabled. |

## Runtime Components

`sim_bringup.launch.py` starts:

- `robot_state_publisher` using `robot_core/urdf/minibase_sim.xacro`.
- Gazebo Harmonic through `ros_gz_sim`.
- `ros_gz_sim create` to spawn the rover.
- `ros_gz_bridge parameter_bridge` with `config/ros_gz_bridge.yaml`.
- `imu_filter_madgwick_node`.
- `robot_localization/ekf_node`.
- Optional RTAB-Map launch.
- `rtabmap_util/point_cloud_xyz`.
- Optional `path_planning_dynamic` nodes.
- Optional `nmpc_controller`.
- Optional `rover_bt` simulation launch.
- Optional RViz.

## Bridge Topics

Configured in `config/ros_gz_bridge.yaml`:

- `/clock`
- `/cmd_vel_safe`
- `/wheel/odom`
- `/joint_states`
- `/imu/data_raw`
- `/depth_camera/imu`
- `/scan`
- `/depth_camera/image`
- `/depth_camera/depth_image`
- `/depth_camera/camera_info`

The point cloud bridge is intentionally disabled in the YAML. The launch file uses `rtabmap_util/point_cloud_xyz` to publish `/depth_camera/points` from the depth image.
