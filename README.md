# Autonomous Rover

by Jossue Espinoza, Gustavo Garcíareal, Bruno Zamora and Sofía Sánchez

-----------------------------------------------------------------------------

`autonomous_rover` is a ROS2 workspace source repository for a differential-drive autonomous rover. It combines real hardware bringup, Azure Kinect sensing, RTAB-Map localization/mapping, Lanelet2-style path planning, NMPC velocity control, behavior-tree supervision, person following, teleoperation, Gazebo Harmonic simulation, a Qt face/debug UI, and the A.R.E.S. Command Hub web dashboard.

The repository is organized as multiple ROS2 packages. Some packages have complete descriptions in their own manifests or READMEs; packages whose manifests still contain TODO descriptions are documented here with `Pending confirmation` where the code does not make the intent explicit.

## General Architecture

![Autonomous rover architecture](Rover%20flow%20-%20Bloques%20final.png)

## Index

- [Main Features](#main-features)
- [Cloning this Repository](#cloning-this-repository)
- [Automatic Installation](#automatic-installation)
- [Quick Start](#quick-start)
- [Build / Compilation](#build--compilation)
- [Execution](#execution)
- [Launch Files](#launch-files)
- [Services and Actions](#services-and-actions)
- [Parameters and Configuration](#parameters-and-configuration)
- [General Architecture](#general-architecture)
- [Detected ROS2 Packages](#detected-ros2-packages)
- [Package Details](#package-details)
- [License](#license)


## Main Features

- Real rover bringup for two motor profiles: `zlac8015d` and `zlac706`.
- Azure Kinect driver integration with RTAB-Map mapping, SLAM, and localization modes.
- Full-stack launch orchestration through `rover_bringup`.
- BehaviorTree.CPP based autonomy and command arbitration in `rover_bt`.
- Person tracking with YOLOv8, OSNet Re-ID, and reactive Twist output.
- Lanelet/map based path planning with a `navigate_to_goal` action server.
- NMPC controller consuming `/sdv_trajectory` and publishing velocity commands.
- Keyboard and Joy-Con teleoperation.
- Gazebo Harmonic simulation with ROS-Gazebo bridges and optional planning/BT stack.
- Native Qt robot face/debug UI and A.R.E.S. web dashboard.

## Cloning this Repository

```bash
cd ~/colcon_ws/src
git clone githttps://github.com/JossueE/autonomous_rover
cd autonomous_rover
```

## Automatic Installation

This repository includes an installer script:

```bash
chmod +x install.sh
./install.sh
```
It assumes ROS2 is already installed and exits with a clear error if ROS2 cannot be detected.

What `install.sh` does:

- Detects the repository root.
- Checks the operating system and warns when it is not Ubuntu/Linux.
- Detects ROS2 and the active or installed ROS2 distro.
- Verifies `/opt/ros/<distro>/setup.bash`.
- Sources the detected ROS2 setup file.
- Checks for `rosdep`, `colcon`, `python3`, `pip`, and `npm`.
- Offers to install missing installer tools with apt when possible.
- Initializes or updates `rosdep` if applicable.
- Runs `rosdep install --from-paths ... --ignore-src`.
- Detects `requirements.txt` files and asks before running `pip install`.
- Detects the A.R.E.S. Command Hub `package-lock.json` and asks before running `npm install`.
- Detects `ament_python` and `ament_cmake` packages.
- Builds the workspace with `colcon build --symlink-install`.
- Verifies `install/setup.bash`.
- Prints final source and run commands.

Help:

```bash
./install.sh --help
```

## Quick Start

```bash
cd ~/colcon_ws/src
git clone https://github.com/JossueE/autonomous_rover
cd autonomous_rover
chmod +x install.sh
./install.sh
source install/setup.bash
```
## Start Everything With

```bash
ros2 run rover_bringup bringup_all_tui.sh --restart
```

## Build / Compilation


Release-style CMake build:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Execution
### Manual Rover Bringup

Full real-rover stack:

```bash
ros2 launch rover_bringup bringup_all.launch.py
```

Available profiles:

```bash
# Robots with different motor drivers
ros2 launch rover_bringup bringup_all.launch.py robot:=zlac706
ros2 launch rover_bringup bringup_all.launch.py robot:=zlac8015d
```

RTAB-Map modes exposed by `bringup_all.launch.py`:

```bash
ros2 launch rover_bringup bringup_all.launch.py rtabmap_mode:=localization
ros2 launch rover_bringup bringup_all.launch.py rtabmap_mode:=mapping
ros2 launch rover_bringup bringup_all.launch.py rtabmap_mode:=slam
```

### Hardware Only

```bash
# You can define the ports of your wheels connection
ros2 launch rover_bringup hardware_bringup.launch.py robot:=zlac706
ros2 launch rover_bringup hardware_bringup.launch.py robot:=zlac8015d motors_port:=/dev/ttyUSB0
```

`hardware_bringup.launch.py` starts the robot state publisher, `joy_node`, `teleop_joycon`, `twist_priority_mux.py`, and the selected `wheels_driver`.

### Localization, Mapping, and SLAM

```bash
ros2 launch rover_bringup k4a_rtabmap.launch.py mode:=localization
ros2 launch rover_bringup k4a_rtabmap.launch.py mode:=mapping
ros2 launch rover_bringup k4a_rtabmap.launch.py mode:=slam
```

The launch file starts `azure_kinect_node`, `imu_filter_madgwick_node`, and `rtabmap_launch/rtabmap.launch.py`.

After creating a `.ply` point cloud from `~/.ros/rtabmap.db`, export and convert it with:

```bash
rtabmap-export --cloud --output_dir /tmp --output rtabmap_cloud ~/.ros/rtabmap.db
pcl_ply2pcd -format 1 /tmp/rtabmap_cloud_cloud.ply ~/.ros/rtabmap_cloud_binary.pcd
```

### Planning and Control

```bash
ros2 launch path_planning_dynamic planning.launch.py robot:=zlac706 use_sim_time:=False
ros2 launch nmpc_controller sim_nmpc.launch.py robot:=zlac706 use_sim_time:=False
```

The planner publishes `/sdv_trajectory`; the NMPC controller consumes that path and publishes `/cmd_vel_nav`. `twist_priority_mux.py` routes `/cmd_vel_safe`, `/cmd_vel_person`, and `/cmd_vel_nav` to `/cmd_vel`.

### Behavior Tree

```bash
ros2 launch rover_bt rover_bt.launch.py
```

Send an external command:

```bash
ros2 service call /rover_bt/send_command rover_bt/srv/SendCommand \
  "{source: 'gui', command: 'navigate', target: 'inicio'}"
```

`rover_bt` also includes detailed operator and technical guides:

- `rover_bt/USER_GUIDE.md`
- `rover_bt/TECHNICAL_GUIDE.md`

### Person Tracking

```bash
ros2 launch person_tracker person_tracker.launch.py mode:=indoor
ros2 launch person_tracker person_tracker.launch.py mode:=outdoor
ros2 launch person_tracker bringup.launch.py robot:=zlac706 mode:=indoor
```

### Simulation

Minimum simulation:

```bash
ros2 launch rover_simulation sim_bringup.launch.py
```

Simulation with RTAB-Map:

```bash
ros2 launch rover_simulation sim_bringup.launch.py use_rtabmap:=true
```

Simulation with planning and behavior tree:

```bash
ros2 launch rover_simulation sim_bringup.launch.py use_rtabmap:=true use_planning:=true use_bt:=true
```

### A.R.E.S. Command Hub

Backend:

```bash
cd front_end/ares-command-hub-main/backend
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
source ../../../install/setup.bash
nice -n 10 python3 -m uvicorn main:app --host 127.0.0.1 --port 8000
```

Frontend:

```bash
cd front_end/ares-command-hub-main
npm run dev -- --host 127.0.0.1 --port 8080
```

The dashboard README is in `front_end/ares-command-hub-main/README.md`.

### Robot Face / Eyes

```bash
ros2 launch eyes_gui robot_face_debug_ui.launch.py
```

This starts `robot_face_debug_ui` and `face_emotion_controller`.

## Launch Files

| Launch file | Package | What it executes | Important parameters | Command |
| --- | --- | --- | --- | --- |
| `bringup_all.launch.py` | `rover_bringup` | Hardware, Azure Kinect/RTAB-Map, planner, NMPC, BT | `robot`, `rtabmap_mode`, `log_level` | `ros2 launch rover_bringup bringup_all.launch.py` |
| `hardware_bringup.launch.py` | `rover_bringup` | URDF publisher, `joy_node`, `teleop_joycon`, twist mux, selected wheels driver | `robot`, `motors_port`, `motor_left_port`, `motor_right_port`, `log_level` | `ros2 launch rover_bringup hardware_bringup.launch.py` |
| `k4a_rtabmap.launch.py` | `rover_bringup` | Azure Kinect, Madgwick IMU filter, RTAB-Map | `mode`, `rtabmap_args`, `log_level` | `ros2 launch rover_bringup k4a_rtabmap.launch.py mode:=slam` |
| `nav2_navigation.launch.py` | `rover_bringup` | Nav2 controller/planner/BT navigator/waypoint follower/lifecycle/RViz | `use_sim_time`, `params_file`, `sensor_topic`, `map_topic`, `odom_topic`, `cmd_vel_topic`, `rviz` | `ros2 launch rover_bringup nav2_navigation.launch.py` |
| `sim_bringup.launch.py` | `rover_simulation` | Gazebo, robot spawn, bridge, EKF, optional RTAB-Map/planning/NMPC/BT/RViz | `use_rtabmap`, `use_planning`, `use_bt`, `use_rviz`, `world`, `spawn_x`, `spawn_y`, `spawn_yaw` | `ros2 launch rover_simulation sim_bringup.launch.py` |
| `rover_bt.launch.py` | `rover_bt` | `rover_bt_node`, `voice_command_node.py`, optional `person_tracker_node` | `params_file`, `tree_xml`, `log_level`, `use_person_tracker`, `person_tracker_mode` | `ros2 launch rover_bt rover_bt.launch.py` |
| `rover_bt_sim.launch.py` | `rover_bt` | BT and voice node with simulation topic overrides | `params_file`, `tree_xml`, `use_sim_time`, `dynamic_waypoints_file` | `ros2 launch rover_bt rover_bt_sim.launch.py` |
| `planning.launch.py` | `path_planning_dynamic` | `pointcloud_roi_node`, `pointcloud_clustering_node`, `path_planning_node`, RViz | `robot`, `use_sim_time`, `rviz_config`, `log_level` | `ros2 launch path_planning_dynamic planning.launch.py` |
| `sim_nmpc.launch.py` | `nmpc_controller` | `nmpc_controller_node` | `robot`, `use_sim_time`, `params_file`, `map_frame`, `base_frame`, `cmd_vel_topic`, `costmap_topic`, `path_topic` | `ros2 launch nmpc_controller sim_nmpc.launch.py` |
| `robot_state_publisher.launch.py` | `robot_core` | `robot_state_publisher` with `minibase_urdf.xacro` | `lidar_model`, `base_color`, `wheel_separation`, `camera_x`, `camera_z`, `camera_pitch` | `ros2 launch robot_core robot_state_publisher.launch.py` |
| `person_tracker.launch.py` | `person_tracker` | `person_tracker_node` | `mode`, `start_enabled` | `ros2 launch person_tracker person_tracker.launch.py` |
| `run_all.launch.py` | `person_tracker` | Azure Kinect and person tracker | `mode` | `ros2 launch person_tracker run_all.launch.py` |
| `bringup.launch.py` | `person_tracker` | Hardware bringup, Kinect/person tracker, image view | `robot`, `mode`, motor ports, `use_image_view` | `ros2 launch person_tracker bringup.launch.py` |
| `uart_lidar.launch.py` | `pacecat` | `pacecat_node`, `pacecat_lidar_crop_node` | `params_file` | `ros2 launch pacecat uart_lidar.launch.py` |
| `robot_face_debug_ui.launch.py` | `eyes_gui` | `robot_face_debug_ui`, `face_emotion_controller` | None declared | `ros2 launch eyes_gui robot_face_debug_ui.launch.py` |
| `benchmark.launch.py` | `odom_comparison` | Hardware, RTAB-Map, wheel odom, benchmark recorder | `robot`, motor ports, `rtabmap_mode`, `output_root`, `log_level` | `ros2 launch odom_comparison benchmark.launch.py` |
| `k4a_device_launch.py` | `azure_kinect_ros2_driver` | `azure_kinect_node` | hardcoded `depth_mode`, `color_resolution` | `ros2 launch azure_kinect_ros2_driver k4a_device_launch.py` |
| `k4a_test_record_launch.py` | `azure_kinect_ros2_driver` | Azure Kinect playback/test subscriber | hardcoded `recording_file` | `ros2 launch azure_kinect_ros2_driver k4a_test_record_launch.py` |

## Services and Actions

| Name | Type | Package | Purpose |
| --- | --- | --- | --- |
| `/rover_bt/send_command` | `rover_bt/srv/SendCommand` | `rover_bt` | External command injection from GUI or ROS service callers. |
| `/rover_bt/save_location` | `rover_bt/srv/SaveLocation` | `rover_bt` | Save the current robot pose as a named location. |
| `navigate_to_goal` | `rover_bt/action/NavigateToGoal` Rover flow - Bloques final.pngion or pose. |
| `/robot_face/set_state` | `eyes_gui/srv/SetFaceState` | `eyes_gui` | Change the robot face state. Accepted states in code: `normal`, `recover`, `crying`, `debug`, `face`, `fullscreen`, `blink`. |
| `/reset_odometry` | `std_srvs/srv/Trigger` | `odometry2` | Reset wheel odometry in `odometry2`. |
| `/odom_compare_recorder/start` | `std_srvs/srv/Trigger` | `odom_comparison` | Start an odometry comparison recording session. |
| `/odom_compare_recorder/stop` | `std_srvs/srv/Trigger` | `odom_comparison` | Stop recording and write benchmark artifacts. |

Custom interfaces:

- `rover_bt/msg/Command.msg`
- `rover_bt/msg/RoverStatus.msg`
- `rover_bt/msg/SensorHealth.msg`
- `rover_bt/srv/SendCommand.srv`
- `rover_bt/srv/SaveLocation.srv`
- `rover_bt/action/NavigateToGoal.action`
- `eyes_gui/srv/SetFaceState.srv`
- `path_planning_dynamic/msg/Obstacle.msg`
- `path_planning_dynamic/msg/ObstacleCollection.msg`
- `path_planning_dynamic/msg/RoadElements.msg`
- `path_planning_dynamic/msg/RoadElementsCollection.msg`

## Parameters and Configuration

| File | Location | Purpose | Important parameters |
| --- | --- | --- | --- |
| `robot_profiles.yaml` | `rover_bringup/config` | Selects robot-specific hardware, geometry, map, planner ROI, and NMPC values. | `driver_package`, motor ports, `wheels_separation`, `wheel_radius`, `max_linear_vel`, `max_angular_vel`, `map_path`. |
| `ekf.yaml` | `rover_bringup/config` | Hardware EKF configuration. | `frequency`, `two_d_mode`, `map_frame`, `odom_frame`, `base_link_frame`, `odom0`, `imu0`. |
| `params.yaml` | `path_planning_dynamic/config` | Planner, local ROI, clustering, and global planner parameters. | `kinematics.model`, `planner.safe_clear`, `planner.obstacle_inflation_radius_cells`, `map_path`, `global_planner_occupancy_output_topic`. |
| `sim_nmpc.yaml` | `nmpc_controller/config` | NMPC tuning. | `h`, `N`, `L`, `v_max`, `a_max`, `d_safe`, `cmd_vel_topic`, `costmap_topic`, `path_topic`. |
| `rover_bt_params.yaml` | `rover_bt/config` | Behavior tree, watchdog, command, TTS, topic, and waypoint defaults. | `bt_tick_rate`, command/status topics, sensor watchdog timeouts, TTS settings. |
| `waypoints.yaml`, `waypoints_sim.yaml` | `rover_bt/config` | Named BT navigation locations. | Location names and map-frame poses. |
| `models.yml` | `rover_bt/config` | Voice/TTS model settings. | Model paths/settings used by voice scripts. |
| `person_tracker_params_indoor.yaml`, `person_tracker_params_outdoor.yaml` | `person_tracker/config` | Indoor/outdoor person tracking profiles. | RGB/depth topics, tracking, velocity, recovery, and obstacle parameters. |
| `ros_gz_bridge.yaml` | `rover_simulation/config` | Gazebo Harmonic bridge mapping. | `/clock`, `/cmd_vel_safe`, `/wheel/odom`, `/joint_states`, `/imu/data_raw`, `/depth_camera/*`, `/scan`. |
| `ekf_sim.yaml` | `rover_simulation/config` | Simulation EKF configuration. | Fuses simulated wheel odom and IMU into `/odom`. |
| `uart_lidar.yaml` | `pacecat/params` | Pacecat LiDAR driver parameters. | Used by `uart_lidar.launch.py`. |
| `trials.yaml` | `odom_comparison/config` | Benchmark trial definitions. | Trial names and benchmark metadata. |
| `nav.rviz`, `sim.rviz` | `rover_bringup/rviz` | RViz views for real navigation and simulation. | Displays are configured inside RViz files. |


```text
autonomous_rover/
|-- azure_kinect_ros2_driver/   # Azure Kinect ROS2 driver and test launchers
|-- eyes_gui/                   # Qt robot face and embedded RViz/debug UI
|-- front_end/                  # A.R.E.S. Command Hub web dashboard
|-- nmpc_controller/            # NMPC velocity controller
|-- odom_comparison/            # Wheel odom vs RTAB-Map benchmark tools
|-- odometry2/                  # Wheel odometry and IMU nodes
|-- pacecat/                    # Pacecat/Bluesea LiDAR driver and crop node
|-- path_planning_dynamic/      # Dynamic path planner, point cloud processing, custom msgs
|-- person_tracker/             # YOLOv8/OSNet person following node
|-- potential_field/            # Potential-field navigation node; Pending confirmation
|-- robot_core/                 # Robot URDF/Xacro and core utility nodes
|-- rover_bringup/              # Real rover launch orchestration, profiles, RViz, maps
|-- rover_bt/                   # Behavior tree brain, voice command node, custom interfaces
|-- rover_simulation/           # Gazebo Harmonic simulation, bridges, worlds
|-- teleop/                     # Keyboard and Joy-Con teleoperation
|-- zlac706_driver2_cpp/        # ZLAC706 dual-controller wheel driver
|-- zlac8015d_driver2_cpp/      # ZLAC8015D dual-motor Modbus driver
|-- install.sh                  # Conservative installer for ROS2 workspace setup
`-- README.md
```


## Detected ROS2 Packages

| Package | Type | Purpose | Build system | Key files | README |
| --- | --- | --- | --- | --- | --- |
| `azure_kinect_ros2_driver` | hardware driver | Azure Kinect depth camera driver | `ament_cmake` + Python install | `src/k4a_ros_node.cpp`, `launch/k4a_device_launch.py`, `urdf/azure_kinect.urdf.xacro` | Yes |
| `eyes_gui` | UI/interface package | Native Qt robot face and embedded RViz debug UI | `ament_cmake` | `src/main.cpp`, `srv/SetFaceState.srv`, `launch/robot_face_debug_ui.launch.py` | Yes |
| `nmpc_controller` | controller | NMPC controller for differential drive robot using CasADi and ROS2 | `ament_cmake` | `src/nmpc_controller_node.cpp`, `config/sim_nmpc.yaml`, `launch/sim_nmpc.launch.py` | Yes |
| `odom_comparison` | benchmark tool | Compares wheel odometry against RTAB-Map odometry | `ament_python` | `odom_comparison/recorder.py`, `launch/benchmark.launch.py`, `config/trials.yaml` | Yes |
| `odometry2` | odometry | Computes differential-drive wheel odometry from encoder ticks | `ament_cmake` | `src/odometry2.cpp`, `src/imu.cpp` | Yes |
| `pacecat` | hardware driver | Pacecat/Bluesea LiDAR package | `ament_cmake` | `src/node.cpp`, `src/pacecat_lidar_crop_node.cpp`, `params/uart_lidar.yaml` | Yes |
| `path_planning_dynamic` | planner/interface package | Dynamic path planner and point cloud obstacle processing; Pending confirmation in manifest | `ament_cmake` + `rosidl` | `src/path_planning.cpp`, `config/params.yaml`, `msg/*.msg` | Yes |
| `person_tracker` | perception/control | Person following with YOLOv8 detection and OSNet Re-ID | `ament_python` | `person_tracker/person_tracker_node.py`, `config/*.yaml`, `launch/*.launch.py` | Yes |
| `potential_field` | planner/controller | Pending confirmation | `ament_cmake` | `src/potentialF.cpp` | Yes |
| `robot_core` | robot description/core | URDF/Xacro and core robot utility nodes; Pending confirmation in manifest | `ament_cmake` | `urdf/*.xacro`, `src/amcl_robot_pose.cpp`, `launch/robot_state_publisher.launch.py` | Yes |
| `rover_bringup` | bringup/launch package | Real rover launch orchestration, robot profiles, maps, RViz | `ament_cmake` | `launch/*.launch.py`, `config/robot_profiles.yaml`, `scripts/bringup_all_tui.sh` | Yes |
| `rover_bt` | behavior tree/interface package | Central behavior tree, command arbitration, watchdogs, voice node | `ament_cmake` + `rosidl` | `src/rover_bt_node.cpp`, `trees/rover_bt_main.xml`, `srv`, `msg`, `action` | Yes |
| `rover_simulation` | simulation | Gazebo Harmonic simulation bringup for minibase rover | `ament_cmake` | `launch/sim_bringup.launch.py`, `config/ros_gz_bridge.yaml`, `worlds/depot.sdf` | Yes |
| `teleop` | teleoperation | Keyboard and Joy-Con teleoperation | `ament_python` | `teleop/teleop_keyboard.py`, `teleop/teleop_joycon.py` | Yes |
| `zlac706_driver2_cpp` | hardware driver | ZLAC706 wheel driver; manifest purpose pending confirmation | `ament_cmake` | `src/wheels_driver.cpp`, `include/zlac706_driver.h` | Yes |
| `zlac8015d_driver2_cpp` | hardware driver | ZLAC8015D dual-motor controller driver over Modbus RTU | `ament_cmake` | `src/wheels_driver.cpp`, `src/zlac8015d_driver/test_driver.cpp` | Yes |

## Package Details

### `rover_bringup`

- Purpose: Launch orchestration for the real rover.
- Nodes/scripts: `bringup_all_tui.sh`, `twist_priority_mux.py`.
- Launch files: `bringup_all.launch.py`, `hardware_bringup.launch.py`, `k4a_rtabmap.launch.py`, `nav2_navigation.launch.py`.
- Key topics from code: `/cmd_vel_safe`, `/cmd_vel_person`, `/cmd_vel_nav`, `/cmd_vel`, `/k4a/*`, `/rtabmap/*`.
- Config: `robot_profiles.yaml`, `ekf.yaml`, maps, RViz files.

### `rover_bt`

- Purpose: Central behavior-tree brain.
- Nodes/scripts: `rover_bt_node`, `voice_command_node.py`, `install_voice_assets.sh`.
- Launch files: `rover_bt.launch.py`, `rover_bt_sim.launch.py`.
- Publishes/subscribes in code: `/cmd_vel_safe`, `/rover_bt/status`, `/person_tracker/enable`, `/person_tracker/profile`, `/odom`, `/rtabmap/odom_info_lite`, `/amcl_robot_pose`, `/rtabmap/localization_pose`, `/scan`, `/k4a/points2`, `/k4a/imu`, `wheel/left_data`, `wheel/right_data`, `/sdv_trajectory`, `/rover_bt/commands`, `/joy`, `/person_tracker/status`, `/rover_bt/tts/say`.
- Services/actions: `/rover_bt/send_command`, `/rover_bt/save_location`, action client `navigate_to_goal`.
- More detail: `rover_bt/USER_GUIDE.md`, `rover_bt/TECHNICAL_GUIDE.md`.

### `path_planning_dynamic`

- Purpose: Planner and point cloud obstacle pipeline; manifest description is pending confirmation.
- Nodes: `path_planning_node`, `pointcloud_clustering_node`, `pointcloud_roi_node`, `junction_gap_check`.
- Launch file: `planning.launch.py`.
- Action server: `navigate_to_goal`.
- Topics from code/config: `/sdv_trajectory`, `/all_available_paths`, `/global_planner`, `/global_planner_occupancy_grid`, `/occupancy_grid_obstacles`, `/robot_footprint_polygon`, `/points_rotated`, `/points_rotated_notground`, `/k4a/points2`.
- Custom messages: `Obstacle`, `ObstacleCollection`, `RoadElements`, `RoadElementsCollection`.

### `nmpc_controller`

- Purpose: NMPC controller for differential drive robot using CasADi and ROS2.
- Nodes/scripts: `nmpc_controller_node`, `path_drawer.py`.
- Launch file: `sim_nmpc.launch.py`.
- Topics from config/code: subscribes to `/sdv_trajectory` and `/occupancy_grid_obstacles`, publishes `/cmd_vel_nav`.
- Config: `config/sim_nmpc.yaml`.

### `person_tracker`

- Purpose: Person following rover node using YOLOv8 detection and OSNet Re-ID.
- Node: `person_tracker_node`.
- Launch files: `person_tracker.launch.py`, `run_all.launch.py`, `bringup.launch.py`.
- Topics from code/config: subscribes to RGB/depth/camera info, `/k4a/imu`, `/person_tracker/enable`, `/person_tracker/profile`; publishes `/cmd_vel_person`, `/person_tracker/person_detected`, `/person_tracker/person_bbox`, `/person_tracker/detections_image`, `/person_tracker/status`.
- Config: indoor and outdoor YAML profiles.

### `teleop`

- Purpose: Keyboard and Joy-Con teleoperation.
- Console scripts: `teleop_keyboard`, `teleop_joycon`.
- Topics from code: keyboard publishes `cmd_vel_safe`; Joy-Con subscribes to `joy` and publishes `cmd_vel_safe`.
- Parameters: `max_lin_vel`, `max_ang_vel`, `axis_linear`, `axis_angular`.

### `rover_simulation`

- Purpose: Gazebo Harmonic simulation bringup.
- Launch file: `sim_bringup.launch.py`.
- World/config: `worlds/depot.sdf`, `config/ros_gz_bridge.yaml`, `config/ekf_sim.yaml`.
- Topics from bridge/config: `/clock`, `/cmd_vel_safe`, `/wheel/odom`, `/joint_states`, `/imu/data_raw`, `/depth_camera/imu`, `/scan`, `/depth_camera/image`, `/depth_camera/depth_image`, `/depth_camera/camera_info`.

### `robot_core`

- Purpose: Robot description and core utility nodes; manifest description is pending confirmation.
- Nodes/scripts: `amcl_robot_pose`, `teleop_keyboard.py`.
- Launch file: `robot_state_publisher.launch.py`.
- URDF/Xacro: `minibase_urdf.xacro`, `minibase_sim.xacro`, `minibase_old_urdf.xacro`.
- Topics from code: publishes `amcl_robot_pose`.

### `odometry2`

- Purpose: Differential-drive wheel odometry from encoder ticks.
- Nodes: `odometry2`, `imu`.
- Topics from code: subscribes `wheel/left_data`, `wheel/right_data`; publishes `wheel/odom` and optionally TF `odom -> base_footprint`; IMU node publishes `imu/data`.
- Service: `/reset_odometry`.

### `zlac706_driver2_cpp`

- Purpose: ZLAC706 driver package. The manifest description is pending confirmation; the README describes two single-motor controllers over serial communication.
- Node: `wheels_driver`.
- Topics from code: subscribes `cmd_vel_safe`, publishes `wheel/left_data`, `wheel/right_data`.
- See `zlac706_driver2_cpp/README.md`.

### `zlac8015d_driver2_cpp`

- Purpose: ZLAC8015D dual-motor controller driver over Modbus RTU.
- Nodes: `wheels_driver`, `test_driver`.
- Topics from code: subscribes `cmd_vel_safe`, publishes `wheel/left_data`, `wheel/right_data`.
- Dependency from manifest: `libmodbus-dev`.
- See `zlac8015d_driver2_cpp/README.md`.

### `eyes_gui`

- Purpose: Native Qt robot face and embedded RViz debug UI.
- Nodes: `robot_face_debug_ui`, `face_emotion_controller`.
- Service: `/robot_face/set_state`.
- Topics from code: publishes `/robot_face/current_state`; face emotion controller subscribes to `/rover_bt/status` and `/sdv_trajectory`.
- Launch file: `robot_face_debug_ui.launch.py`.

### `odom_comparison`

- Purpose: Independent benchmark for wheel odometry versus RTAB-Map odometry.
- Console scripts: `odom_compare_recorder`, `trial_runner`, `topic_guard`.
- Launch file: `benchmark.launch.py`.
- Topics from code: `/wheel/odom`, `/rtabmap/odom`, `/rtabmap/odom_info_lite`, `/rtabmap/odom_info`.
- Services: `/odom_compare_recorder/start`, `/odom_compare_recorder/stop`.

### `azure_kinect_ros2_driver`

- Purpose: ROS2 driver for the Azure Kinect depth camera sensor.
- Nodes/scripts: `azure_kinect_node`, `test_azure_kinect_subscriber.py`.
- Launch files: `k4a_device_launch.py`, `k4a_test_record_launch.py`.
- Topics are produced by the driver and consumed elsewhere as `/k4a/rgb/image_raw`, `/k4a/depth_to_rgb/image_raw`, `/k4a/rgb/camera_info`, `/k4a/points2`, and `/k4a/imu`.

### `pacecat`

- Purpose: Pacecat/Bluesea LiDAR driver package.
- Nodes: `pacecat_node`, `pacecat_client`, `pacecat_lidar_crop_node`.
- Launch file: `uart_lidar.launch.py`.
- Topics from launch: crops `scan_raw` to `scan`.

### `potential_field`

- Purpose: Pending confirmation.
- Node: `potential_field_node`.
- Source: `src/potentialF.cpp`.

## License

The root repository license is MIT. See `LICENSE`.
