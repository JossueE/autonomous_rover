# Azure Kinect ROS2 Driver

This repo is based in the work from: [ckennedy2050](https://github.com/ckennedy2050/Azure_Kinect_ROS2_Driver)

This project is a ROS2 node which publishes sensor data from the [Azure Kinect Developer Kit](https://azure.microsoft.com/en-us/services/kinect-dk/) to the [Robot Operating System (ROS)](https://docs.ros.org/). Developers working with ROS2 can use this node to connect an Azure Kinect Developer Kit to an existing ROS2 installation.

This repository uses the [Azure Kinect Sensor SDK](https://github.com/microsoft/Azure-Kinect-Sensor-SDK) to communicate with the Azure Kinect DK. It supports Linux installations of ROS2.

This is a port of the ROS1 drivers:

- [github.com/matlabbe/Azure_Kinect_ROS_Driver](https://github.com/matlabbe/Azure_Kinect_ROS_Driver)
- [github.com/microsoft/Azure_Kinect_ROS_Driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver)

## Features

This ROS node outputs a variety of sensor data, including:

- A PointCloud2, optionally colored using the color camera
- Raw color, depth and infrared Images, including CameraInfo messages containing calibration information
- Rectified depth Images in the color camera resolution
- Rectified color Images in the depth camera resolution
- The IMU sensor stream
- A TF2 model representing the extrinsic calibration of the camera

The camera is fully configurable using a variety of options which can be specified in ROS2 launch files or on the command line.

However, this node does ***not*** expose all the sensor data from the Azure Kinect Developer Kit hardware. It does not provide access to:

- Microphone array
- Body tracking

## Status

This is a basic port to ROS2 and not thoroughly tested at this point. Limited testing has been performed with 
ROS2 Jazzy on Ubuntu 24.04.

Community additions and fixes are welcome.

## Building

### Azure Kinect Sensor SDK

Follow the [official installation instructions](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/blob/develop/docs/usage.md#Installation) in the Azure Kinect Sensor SDK repo to install the sensor SDK for your platform.

First check if your computer is ARM64 or AMD64:

```bash
cat /etc/os-release
uname -m
```

### AMD64

```bash
curl -sSL -O https://packages.microsoft.com/config/ubuntu/18.04/packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
rm packages-microsoft-prod.deb

sudo apt-get update
sudo apt-get install -y libk4a1.4 libk4a1.4-dev

sudo wget -O /etc/udev/rules.d/99-k4a.rules \
  https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG plugdev $USER
```

### ARM64

```bash
curl -sSL -O https://packages.microsoft.com/config/ubuntu/18.04/multiarch/packages-microsoft-prod.deb
sudo dpkg -i packages-microsoft-prod.deb
rm packages-microsoft-prod.deb

sudo apt-get update
sudo apt-get install -y libk4a1.4 libk4a1.4-dev 

sudo wget -O /etc/udev/rules.d/99-k4a.rules \
  https://raw.githubusercontent.com/microsoft/Azure-Kinect-Sensor-SDK/develop/scripts/99-k4a.rules

sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG plugdev $USER
```

### Compiling

Clone the repo into the `src` directory of your [ROS2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

```bash
colcon build --symlink-install --packages-select azure_kinect_ros2_driver
```

## Running
Source your workspace: `source <ROS2 ws>/install/setup.bash`

`ros2 run azure_kinect_ros2_driver azure_kinect_node`

To enable point cloud publishing (disabled by default for performance):

`ros2 run azure_kinect_ros2_driver azure_kinect_node --ros-args -p point_cloud:=true`

For colored point cloud (requires color camera enabled):

`ros2 run azure_kinect_ros2_driver azure_kinect_node --ros-args -p point_cloud:=true -p rgb_point_cloud:=true`

or from a file recorded via `k4arecorder`:

`ros2 run azure_kinect_ros2_driver azure_kinect_node --ros-args -p recording_file:=/path/to/myrecording.mkv`

A simple Python-based subscriber is included to visualize some of the data being published. An `image_proc` node is used
to undistort the images. Install via `sudo apt install ros-<$ROS2_DISTRO>-image-pipeline`

Launch the nodes:

`ros2 launch azure_kinect_ros2_driver k4a_test_record_launch.py`

## Parameters

The node supports various parameters to configure the camera and output. Parameters can be set via command line with `--ros-args -p param:=value` or in launch files.

### Camera Configuration
- `sensor_sn` (string, default: ""): Serial number of the sensor to use (for multi-sensor setups)
- `depth_enabled` (bool, default: true): Enable depth camera
- `depth_mode` (string, default: "NFOV_UNBINNED"): Depth mode (NFOV_UNBINNED, NFOV_BINNED, WFOV_UNBINNED, WFOV_BINNED, PASSIVE_IR)
- `color_enabled` (bool, default: true): Enable color camera
- `color_format` (string, default: "bgra"): Color format (bgra, jpeg)
- `color_resolution` (string, default: "1536P"): Color resolution (720P, 1080P, 1440P, 1536P, 2160P, 3072P)
- `fps` (int, default: 30): Frame rate (5, 15, 30)

### Output Configuration
- `point_cloud` (bool, default: false): Enable point cloud publishing
- `rgb_point_cloud` (bool, default: false): Enable colored point cloud (requires point_cloud=true and color_enabled=true)
- `point_cloud_in_depth_frame` (bool, default: true): Generate point cloud in depth frame coordinates
- `rescale_ir_to_mono8` (bool, default: false): Rescale IR images to mono8
- `ir_mono8_scaling_factor` (float, default: 1.0): Scaling factor for IR mono8 rescaling

### Recording and Playback
- `recording_file` (string, default: ""): Path to MKV file for playback
- `recording_loop_enabled` (bool, default: false): Loop playback of recording

### Synchronization
- `wired_sync_mode` (int, default: 0): Wired sync mode (0=standalone, 1=master, 2=subordinate)
- `subordinate_delay_off_master_usec` (int, default: 0): Delay for subordinate mode

### IMU and Other
- `imu_rate_target` (int, default: 100): Target IMU rate
- `body_tracking_enabled` (bool, default: false): Enable body tracking (not implemented)
- `tf_prefix` (string, default: ""): Prefix for TF frame names

## Topics Published

When running, the node publishes the following topics (prefix `/k4a/` by default):

- `/k4a/rgb/image_raw` (sensor_msgs/Image): Raw color image
- `/k4a/rgb/camera_info` (sensor_msgs/CameraInfo): Color camera calibration
- `/k4a/depth/image_raw` (sensor_msgs/Image): Raw depth image
- `/k4a/depth/camera_info` (sensor_msgs/CameraInfo): Depth camera calibration
- `/k4a/ir/image_raw` (sensor_msgs/Image): Raw IR image
- `/k4a/points2` (sensor_msgs/PointCloud2): Point cloud (if enabled)
- `/k4a/imu` (sensor_msgs/Imu): IMU data
- TF frames for camera transforms

## Visualization

To visualize the data:

1. **RViz2**: Run `rviz2`, set Fixed Frame to `azure_kinect_rgb_camera_link`, add Image and PointCloud2 displays.

2. **Test Subscriber**: Use the included Python subscriber:
   ```bash
   ros2 run azure_kinect_ros2_driver test_azure_kinect_subscriber.py
   ```

3. **Image Processing**: For rectified images, use image_proc:
   ```bash
   ros2 run image_proc image_proc --ros-args -r image:=/k4a/rgb/image_raw -r camera_info:=/k4a/rgb/camera_info
   ```

## License

[MIT License](LICENSE)
