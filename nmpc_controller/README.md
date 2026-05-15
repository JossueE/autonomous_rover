# nmpc_controller
by Jossue Espinoza

Nonlinear Model Predictive Control (NMPC) package for a differential-drive robot using ROS 2 and CasADi/IPOPT.

This package provides:
- a reusable NMPC core library
- a ROS 2 controller node that follows a `nav_msgs/Path`
- a simple `path_drawer.py` tool to draw paths from RViz clicked points

## Prerequisites

- A ROS 2 workspace built with `colcon`
- ROS 2 core packages installed
- TurtleBot3 packages if you want to run the minimal TurtleBot example

## Setup

### 1. Install system dependencies

```bash
sudo apt update
sudo apt install -y build-essential cmake gfortran git pkg-config liblapack-dev
```

### 2. Build and install CasADi

This package looks for CasADi manually through `CASADI_INCLUDE_DIR` and `CASADI_LIBRARY`.

```bash
cd /tmp
git clone https://github.com/casadi/casadi.git
cd casadi
rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DWITH_IPOPT=ON \
  -DWITH_BUILD_IPOPT=ON \
  -DWITH_BUILD_MUMPS=ON \
  -DWITH_BUILD_REQUIRED=ON

cmake --build build -j"$(nproc)"
sudo cmake --install build

echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/casadi.conf
sudo ldconfig
```

### 3. Build the package

From the root of your ROS 2 workspace:

```bash
colcon build --packages-select nmpc_controller 

```

Then source the workspace:

```bash
source install/setup.bash
```

## Package Contents

- `src/nmpc_controller/nmpc_controller.cpp`
  Core NMPC formulation and CasADi/IPOPT solve logic.
- `src/nmpc_controller_node.cpp`
  ROS 2 node that subscribes to a path and publishes `cmd_vel`.
- `scripts/path_drawer.py`
  Helper tool to create a path from RViz clicked points and publish a simple empty costmap.

## Topics

### `nmpc_controller_node`

Subscriptions:
- `path_topic` (`nav_msgs/Path`)
- `costmap_topic` (`nav_msgs/OccupancyGrid`)

Publications:
- `cmd_vel_topic` (`geometry_msgs/Twist`)

TF:
- looks up the transform from `map_frame` to `base_frame`

### `path_drawer.py`

Subscriptions:
- `/clicked_point` (`geometry_msgs/PointStamped`)
- `/odom` (`nav_msgs/Odometry`)

Publications:
- `/drawn_plan_raw` (`nav_msgs/Path`)
- `/drawn_plan` (`nav_msgs/Path`)
- `/move_base/local_costmap/costmap` (`nav_msgs/OccupancyGrid`)

Services:
- `/clear_drawn_plan` (`std_srvs/srv/Empty`)

## Parameters

### Controller parameters

- `h`
  Sampling time in seconds.
- `N`
  Prediction horizon length in samples.
- `L`
  Differential-drive track width in meters.
- `v_max`
  Maximum wheel speed in meters per second.
- `a_max`
  Maximum wheel acceleration in meters per second squared.
- `lambda_1`
  Weight for smooth acceleration changes.
- `lambda_theta`
  Weight for heading tracking error.
- `lambda_v`
  Weight for wheel-speed tracking error.
- `d_safe`
  Minimum obstacle clearance in meters.
- `voxel_size`
  Obstacle voxel size in meters.
- `max_range`
  Obstacle search range in meters.
- `max_obstacles`
  Fixed number of obstacle slots passed to the NLP.

### ROS interface parameters

- `map_frame`
  Global frame used for control and TF lookup.
- `base_frame`
  Robot body frame.
- `cmd_vel_topic`
  Output velocity command topic.
- `costmap_topic`
  Occupancy grid input topic.
- `path_topic`
  Path reference topic.
- `goal_tolerance`
  Distance threshold used to declare the goal reached.

## Running the Controller

Start the controller node with the default parameters:

```bash
ros2 run nmpc_controller nmpc_controller_node
```

Or override parameters from the command line:

```bash
ros2 run nmpc_controller nmpc_controller_node --ros-args \
  -p map_frame:=odom \
  -p base_frame:=base_footprint \
  -p cmd_vel_topic:=/cmd_vel \
  -p costmap_topic:=/move_base/local_costmap/costmap \
  -p path_topic:=/drawn_plan \
  -p goal_tolerance:=0.05
```

## Minimal TurtleBot3 Example

This is the smallest end-to-end example to test the package with TurtleBot3.

### 1. Set the TurtleBot3 model

```bash
export TURTLEBOT3_MODEL=burger
```

### 2. Launch an empty Gazebo world

```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 3. Open RViz

```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

### 4. Start the path drawer

```bash
ros2 run nmpc_controller path_drawer.py
```

### 5. Start the NMPC controller

```bash
ros2 run nmpc_controller nmpc_controller_node --ros-args \
  -p map_frame:=odom \
  -p base_frame:=base_footprint \
  -p cmd_vel_topic:=/cmd_vel \
  -p costmap_topic:=/move_base/local_costmap/costmap \
  -p path_topic:=/drawn_plan
```

### 6. Draw a path in RViz

Use the `Publish Point` tool in RViz:
- the first click starts a path anchored at the current robot pose
- each additional click extends the same path
- the smoothed path is published on `/drawn_plan`

### 7. Clear the path

```bash
ros2 service call /clear_drawn_plan std_srvs/srv/Empty "{}"
```

## Notes

> [!NOTE]
> - `path_drawer.py` transforms clicked points into the `odom` frame before publishing the path.
> - The controller keeps track of progress along the path, so updating the path with new clicks is more robust than restarting from the beginning.
> - The controller uses a differential-drive model, not Ackermann.
> - This package currently publishes velocity commands as `geometry_msgs/Twist`.
> - For TurtleBot setups that expect stamped velocity commands, you may need to adapt the interface from `geometry_msgs/Twist` to `geometry_msgs/TwistStamped`.

## Troubleshooting

### CasADi not found at configure time

Build again and pass the library/include paths explicitly:

```bash
colcon build --packages-select nmpc_controller \
  --cmake-args \
  -DCASADI_INCLUDE_DIR=/usr/local/include \
  -DCASADI_LIBRARY=/usr/local/lib/libcasadi.so
```

### The robot stops before the last waypoint

Check:
- `goal_tolerance`
- `map_frame` and `base_frame`
- that the path is being published in the expected frame
- that the robot is receiving valid TF and `cmd_vel`

### Clicked points appear shifted

Make sure RViz has valid TF between the clicked-point frame and `odom`. The path drawer transforms every clicked point to `odom` before publishing.
