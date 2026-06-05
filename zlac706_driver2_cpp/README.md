# ZLAC706 Driver - ROS2 Package

This package provides a C++ driver and ROS2 node for controlling two ZLAC706 single-motor controllers over serial communication (usually via RS232 to USB adapters).

## Package Structure

- **Driver Library** (`zlac706_driver.h/cpp`, `AsyncSerial.h/cpp`, `wheels_writer.h/cpp`): Standalone serial interface to the ZLAC706 hardware using Boost Asio.
- **ROS2 Node** (`wheels_driver.cpp`): ROS2 wrapper providing differential drive velocity control via `geometry_msgs/msg/Twist`. It manages two independent ZLAC706 controllers (Left and Right).

---

## Installation

### Prerequisites

This package uses Boost for asynchronous serial communication (no `libmodbus` is required for the 706).

```bash
sudo apt-get update
sudo apt-get install -y libboost-system-dev libboost-thread-dev
```
> This apt-requirement is implicit in ROS2 installation

### Clone

```bash
cd ~/colcon_ws/src
git clone https://github.com/JossueE/zlac706_driver2_cpp
```

---

## Usage

### Build

```bash
cd ~/colcon_ws
colcon build --packages-select zlac706_driver2_cpp
source install/setup.bash
```

### Launch the node

Since this is a differential drive controller using two independent ZLAC706 drivers, you must provide **two** serial ports (one for the Left wheel, one for the Right wheel) as arguments.

```bash
ros2 run zlac706_driver2_cpp wheels_driver /dev/ttyUSB0 /dev/ttyUSB1
```
> If you don't know the serial ports, you can use the following command to find them:

```bash
ls /dev/ttyUSB*
```
> If USB ports don't have permissions, you can use the following command to give them:

```bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

### Send velocity commands

Publish to the `cmd_vel_safe` topic to move the robot:

```bash
ros2 topic pub /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"
```

### Monitor wheel data

View the raw encoder ticks published by the driver:

```bash
ros2 topic echo /wheel/left_data
ros2 topic echo /wheel/right_data
```

---

## Configuration

You can customize the driver parameters via command line or using a ROS2 launch file.

```bash
ros2 run zlac706_driver2_cpp wheels_driver /dev/ttyUSB0 /dev/ttyUSB1 \
    --ros-args \
    -p baudrate:=57600 \
    -p wheel_radius:=0.1 \
    -p wheels_separation:=0.315 \
    -p wheelL_is_backward:=true \
    -p wheelR_is_backward:=false \
    -p accel_time_ms:=100 \
    -p decel_time_ms:=100 \
    -p unlock_driver:=true \
    -p time_disabled_driver_s:=3.0
```

### Key Parameters:
- `wheel_radius`: Radius of the wheels in meters.
- `wheels_separation`: Distance between the two wheels in meters.
- `wheelL_is_backward` / `wheelR_is_backward`: Invert motor direction if needed.
- `accel_time_ms` / `decel_time_ms`: Hardware acceleration and deceleration times (Minimum 100 ms).
- `unlock_driver` and `time_disabled_driver_s`: If `unlock_driver` is true, keeping a 0 speed command for `time_disabled_driver_s` seconds will unlock the wheels (disable holding torque).

---

## Driver Notes

> [!IMPORTANT]
> The ZLAC706 driver uses a custom hexadecimal serial protocol, unlike the Modbus RTU used by the 8015D. You must ensure your serial adapters are configured to the correct baudrate (default is 57600). The speed resolution steps are omitted as they do not apply to this driver version.

### Recommended startup sequence
1. Connect power to the ZLAC706 controllers.
2. Connect your serial communication adapters (e.g., `/dev/ttyUSB0` and `/dev/ttyUSB1`).
3. Ensure you have the permissions to read/write to the serial ports (e.g., `sudo usermod -aG dialout $USER`).
4. Launch the `wheels_driver` node and provide **both** port paths.

### Safety & Timeout
- If `cmd_vel_safe` has more than 1 publisher, the driver node will print a fatal error and shut down.
- If there are zero publishers on `cmd_vel_safe`, the driver will automatically stop the robot and lock the wheels for security.
- Always send a `0.0` Twist command to peacefully stop the motors. When the node is destroyed (e.g., via `Ctrl+C`), it automatically attempts to send a stop command to the hardware.
