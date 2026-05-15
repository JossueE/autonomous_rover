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

- Start IMU filter (Madgwick). This remaps the raw IMU and publishes a filtered IMU topic used by RTAB-Map:

```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node \
  --ros-args \
  -p use_mag:=false -p world_frame:=enu -p publish_tf:=false \
  -r imu/data_raw:=/k4a/imu -r imu/data:=/k4a/imu_filtered
```

- Start robot hardware bringup (motor controllers, tf publishers, etc.):

```bash
ros2 launch rover_bringup hardware_bringup.launch.py use_wheel_odometry:=false
```


3) Launch RTAB-Map
=======
ros2 launch rtabmap_launch rtabmap.launch.py   rtabmap_args:="--delete_db_on_start \
  --Reg/Force3DoF true \
  --Grid/FromDepth false \
  --Grid/3D false \
  --Grid/RangeMax 4.5 \
  --Grid/MaxGroundHeight 0.10 \
  --Grid/MaxObstacleHeight 1.20 \
  --Grid/CellSize 0.05"   rgb_topic:=/k4a/rgb/image_raw   depth_topic:=/k4a/depth_to_rgb/image_raw   camera_info_topic:=/k4a/rgb/camera_info   scan_cloud_topic:=/k4a/points2   subscribe_scan_cloud:=true   imu_topic:=/k4a/imu_filtered   wait_imu_to_init:=true   frame_id:=base_footprint   approx_sync:=true   approx_sync_max_interval:=0.02   wait_for_transform:=0.3   queue_size:=20   qos:=2   rviz:=true

- Example launch with commonly tuned arguments. These tune grid generation, 3-DoF registration, synchronization, and topics used by this repository:

```bash
ros2 launch rtabmap_launch rtabmap.launch.py \
  localization:=true \
  rtabmap_args:="\
  --Reg/Force3DoF true \
  --Grid/FromDepth false \
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
  imu_topic:=/k4a/imu_filtered \
  wait_imu_to_init:=true \
  frame_id:=base_footprint \
  approx_sync:=true \
  approx_sync_max_interval:=0.02 \
  wait_for_transform:=0.3 \
  queue_size:=20 \
  qos:=2 \
  rviz:=true \
  #use_sim_time:=true
```


Notes on key RTAB-Map args:
- `--delete_db_on_start`: start fresh each run (useful during initial tuning).
- `--Reg/Force3DoF true`: force 3-DoF (planar) registration when operating on a ground vehicle.
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
