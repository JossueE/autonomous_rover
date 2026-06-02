# autonomous_rover

## Mapping

Overview

This section documents how to run mapping on the rover using the Azure Kinect + RTAB-Map (ROS 2 Jazzy). It lists dependencies, recommended launch order, key RTAB-Map arguments, and commands to save/export maps.

1) Install required packages

```bash
sudo apt update
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-imu-filter-madgwick ros-jazzy-pcl-ros
```

2) Bring up sensors and hardware

- Start the Azure Kinect node (point cloud enabled, RGB point cloud disabled to reduce CPU):

```bash
ros2 run azure_kinect_ros2_driver azure_kinect_node \
  --ros-args -p point_cloud:=true -p rgb_point_cloud:=false -p fps:=30
```

- Start robot hardware bringup. This uses wheel odometry plus the Azure Kinect IMU through Madgwick and the EKF, publishing `/odom` and `odom -> base_footprint`:

```bash
ros2 launch rover_bringup hardware_bringup.launch.py \
  use_wheel_odometry:=true \
  use_imu_odometry:=true
```


3) Launch RTAB-Map

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  localization:=false \
  visual_odometry:=true \
  icp_odometry:=true \
  odom_topic:=odom \
  publish_tf_odom:=true \
  rtabmap_args:="--delete_db_on_start \
  --Reg/Strategy 2 \
  --Reg/Force3DoF true \
  --RGBD/ForceOdom3DoF true \
  --Grid/3D false \
  --Grid/RangeMax 4.5 \
  --Grid/MaxGroundHeight 0.10 \
  --Grid/MaxObstacleHeight 1.20 \
  --Grid/CellSize 0.05" \
  rgb_topic:=/k4a/rgb/image_raw \
  depth_topic:=/k4a/depth_to_rgb/image_raw \
  camera_info_topic:=/k4a/rgb/camera_info \
  scan_cloud_topic:=/k4a/points2 \
  subscribe_scan_cloud:=true \
  imu_topic:=/k4/imu \
  frame_id:=base_footprint \
  approx_sync:=true \
  approx_sync_max_interval:=0.02 \
  wait_for_transform:=2.0 \
  queue_size:=20 \
  qos:=2 \
  qos_odom:=1 \
  rviz:=true
```

- Localization against an existing RTAB-Map database uses the same EKF odometry input:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  localization:=true \
  visual_odometry:=false \
  odom_topic:=/odom \
  publish_tf_odom:=false \
  rtabmap_args:="\
  --Reg/Force3DoF true \
  --RGBD/ForceOdom3DoF true \
  --Grid/3D false \
  --Grid/RangeMax 4.5 \
  --Grid/MaxGroundHeight 0.10 \
  --Grid/MaxObstacleHeight 1.20 \
  --Grid/CellSize 0.05" \
  rgb_topic:=/k4a/rgb/image_raw \
  depth_topic:=/k4a/depth_to_rgb/image_raw \
  camera_info_topic:=/k4a/rgb/camera_info \
  scan_cloud_topic:=/k4a/points2 \
  subscribe_scan_cloud:=true \
  imu_topic:=/imu/data \
  frame_id:=base_footprint \
  approx_sync:=true \
  approx_sync_max_interval:=0.02 \
  wait_for_transform:=2.0 \
  queue_size:=20 \
  qos:=2 \
  qos_odom:=1 \
  rviz:=false
```


Notes on key RTAB-Map args:
- `--delete_db_on_start`: start fresh each run (useful during initial tuning).
- `--Reg/Force3DoF true`: force 3-DoF (planar) registration when operating on a ground vehicle.
- `visual_odometry:=false` and `odom_topic:=/odom`: make RTAB-Map use the EKF odometry instead of creating RGB-D odometry.
- Grid settings (`Grid/*`): control occupancy grid generation and dimensions; tune `RangeMax` and `CellSize` to fit environment.
- `approx_sync` and `approx_sync_max_interval`: allow approximate sensor synchronization; reduce false mismatches from sensor latency.

4) Teleoperation (optional)

> - Keyboard teleop:
> 
> Use the keyboard teleop node to drive while mapping (remapped to safe `cmd_vel` topic):
> 
> ```bash
> ros2 run robot_core teleop_keyboard.py --ros-args -r cmd_vel:=/cmd_vel_safe
> ```
> 
> - Joycon teleop:
> 
> This repository includes `teleop_joycon`, a ROS 2 node that listens on the `/joy` topic and publishes safe velocity commands to `cmd_vel_safe`. It requires a system joystick driver node (commonly `joy_node`) to be running and publishing `/joy` messages.
> 
> 1. Start a joystick driver (example using the common `joy` package):
> 
> ```bash
> # run on the machine with the joystick attached
> ros2 run joy joy_node
> ```
> 
> 2. Run the Joy-Con teleop node (package `teleop`):
> 
> ```bash
> ros2 run teleop teleop_joycon
> ```

5) Save/export maps and point clouds

- Create a directory to store PCD map exports:

```bash
mkdir -p ~/maps/rtabmap_pcd
cd ~/maps/rtabmap_pcd
```

- Export saved point cloud from RTAB-Map to PCD files (topic `/rtabmap/cloud_map`):

```bash
ros2 run pcl_ros pointcloud_to_pcd \
  --ros-args -r input:=/rtabmap/cloud_map
```

¿
- Alternatively, publish the map from RTAB-Map (useful to trigger map saving or to view in RViz):

```bash
ros2 service call /rtabmap/publish_map std_srvs/srv/Empty
```

6) Troubleshooting & tips

- If maps are noisy or drifting, verify IMU orientation and TF frames (IMU should be published and aligned with `base_link`).
- For large environments, disable `--delete_db_on_start` to accumulate a longer-term map, and increase `Grid/RangeMax`.
- Use `rviz` (enabled above) to observe camera, pointcloud, and map topics during tuning.

source install/setup.bash
rviz2 -d install/autonomous_robot_simulation/share/autonomous_robot_simulation/rviz/new.rviz


ros2 launch path_planning_dynamic planning.launch.py use_sim_time:=true

rviz2 -d install/autonomous_robot_simulation/share/autonomous_robot_simulation/rviz/new.rviz


### Notas Jossue 

ros2 launch rover_bringup hardware_bringup.launch.py 

ros2 launch rover_bringup k4a_rtabmap.launch.py mode:=localization

ros2 launch path_planning_dynamic planning.launch.py

░▒▓snorlix ~ ▓▒░
❯  ros2 node list
/path_planning_node
/rtabmap/icp_odometry
░▒▓snorlix ~ ▓▒░
❯  ros2 node list
/ares_command_hub_bridge
/imu_filter_madgwick_node
/k4a_ros2_node
/nmpc_controller_node
/path_planning_node
/pointcloud_clustering_node
/pointcloud_roi_node
/robot_state_publisher
/rtabmap/icp_odometry
/rtabmap/rtabmap
/rtabmap/rtabmap_viz
/rtabmap/transform_listener_impl_5db3feb4e8f0
/rtabmap/transform_listener_impl_5f61b6b35de0
/rtabmap/transform_listener_impl_65195b8f22d0
/rviz2
/transform_listener_impl_5a0eb6a934d8
/transform_listener_impl_5bad5fce0e38
/transform_listener_impl_622919e39a30
/transform_listener_impl_64d16e1fe0c0
/wheels_driver
░▒▓snorlix ~ ▓▒░
❯  ros2 topic list
/all_available_paths
/camera/color/image_raw
/camera/image_raw
/car
/clicked_point
/cmd_vel_safe
/depth_camera/points
/detections
/diagnostics
/global_planner
/global_planner_occupancy_grid
/global_planner_occupancy_grid_updates
/goal_pose
/gps/fix
/image_raw
/imu/data
/initialpose
/joint_states
/k4a/depth/camera_info
/k4a/depth/image_raw
/k4a/depth/image_raw/compressed
/k4a/depth/image_raw/compressedDepth
/k4a/depth/image_raw/theora
/k4a/depth/image_raw/zstd
/k4a/depth_to_rgb/camera_info
/k4a/depth_to_rgb/image_raw
/k4a/depth_to_rgb/image_raw/compressed
/k4a/depth_to_rgb/image_raw/compressedDepth
/k4a/depth_to_rgb/image_raw/theora
/k4a/depth_to_rgb/image_raw/zstd
/k4a/imu
/k4a/imu_filtered
/k4a/ir/camera_info
/k4a/ir/image_raw
/k4a/ir/image_raw/compressed
/k4a/ir/image_raw/compressedDepth
/k4a/ir/image_raw/theora
/k4a/ir/image_raw/zstd
/k4a/points2
/k4a/rgb/camera_info
/k4a/rgb/image_raw
/k4a/rgb/image_raw/compressed
/k4a/rgb/image_raw/compressedDepth
/k4a/rgb/image_raw/theora
/k4a/rgb/image_raw/zstd
/k4a/rgb_to_depth/camera_info
/k4a/rgb_to_depth/image_raw
/k4a/rgb_to_depth/image_raw/compressed
/k4a/rgb_to_depth/image_raw/compressedDepth
/k4a/rgb_to_depth/image_raw/theora
/k4a/rgb_to_depth/image_raw/zstd
/none_real_traj
/obstacle_info
/occupancy_grid_obstacles
/odom
/parameter_events
/points_rotated
/points_rotated_notground
/real_trajectories_option_2
/robot_description
/robot_footprint_polygon
/robot_footprint_polygon_array
/rosout
/rtabmap/apriltag/detections
/rtabmap/aruco/detections
/rtabmap/aruco_opencv/detections
/rtabmap/cloud_ground
/rtabmap/cloud_map
/rtabmap/cloud_obstacles
/rtabmap/global_path
/rtabmap/global_path_nodes
/rtabmap/global_pose
/rtabmap/goal
/rtabmap/goal_node
/rtabmap/goal_reached
/rtabmap/grid_prob_map
/rtabmap/grid_prob_map_updates
/rtabmap/info
/rtabmap/initialpose
/rtabmap/labels
/rtabmap/landmark_detection
/rtabmap/landmark_detections
/rtabmap/landmarks
/rtabmap/local_grid_empty
/rtabmap/local_grid_ground
/rtabmap/local_grid_obstacle
/rtabmap/local_path
/rtabmap/local_path_nodes
/rtabmap/localization_pose
/rtabmap/map
/rtabmap/mapData
/rtabmap/mapGraph
/rtabmap/mapOdomCache
/rtabmap/mapPath
/rtabmap/octomap_binary
/rtabmap/octomap_empty_space
/rtabmap/octomap_full
/rtabmap/octomap_global_frontier_space
/rtabmap/octomap_grid
/rtabmap/octomap_ground
/rtabmap/octomap_obstacles
/rtabmap/octomap_occupied_space
/rtabmap/odom
/rtabmap/odom_filtered_input_scan
/rtabmap/odom_info
/rtabmap/odom_info_lite
/rtabmap/odom_last_frame
/rtabmap/odom_local_map
/rtabmap/odom_local_scan_map
/rtabmap/odom_rgbd_image
/rtabmap/odom_sensor_data/compressed
/rtabmap/odom_sensor_data/features
/rtabmap/odom_sensor_data/raw
/rtabmap/rtabmap/republish_node_data
/scan
/sdv_trajectory
/tf
/tf_static
/user_data_async
/wheel/left_data
/wheel/right_data
░▒▓snorlix ~ ▓▒░
❯  

Backend A.R.E.S. Command Hub:

```bash
cd /home/snorlix/colcon_ws/src/autonomous_rover/front_end/ares-command-hub-main/backend
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
source /home/snorlix/colcon_ws/install/setup.bash
python3 -m uvicorn main:app --host 127.0.0.1 --port 8000
```

Frontend A.R.E.S. Command Hub:

```bash
cd /home/snorlix/colcon_ws/src/autonomous_rover/front_end/ares-command-hub-main
npm run dev -- --host 127.0.0.1 --port 8080
```

# Don not use
rtabmap-export --cloud --output_dir /tmp --output rtabmap_cloud ~/.ros/rtabmap.db
pcl_ply2pcd -format 1 /tmp/rtabmap_cloud_cloud.ply ~/.ros/rtabmap_cloud_binary.pcd
