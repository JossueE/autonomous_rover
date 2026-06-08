
# ZLAC8015D Driver - ROS2 Package

This package provides a C++ driver and ROS2 node for controlling the ZLAC8015D dual-motor controller via Modbus RTU over serial communication.

## Wiring Diagram

<figure style="margin:0; text-align:center; border:1px solid #eaecef; padding:6px; border-radius:6px;">
  <img src="docs/wiring_diagram.png"
       alt="Wiring Diagram"
       style="max-width:100%; height:auto;" />
</figure>

## Package Structure

- **Driver Library** (`zlac8015d_driver.h/cpp`): Standalone Modbus RTU interface to the ZLAC8015D hardware
- **ROS2 Node** (`wheels_driver.cpp`): ROS2 wrapper providing velocity control via `geometry_msgs/Twist`
- **Test Application** (`test_driver.cpp`): Standalone test utility for driver validation

---

## Installation

### Prerequisites

```bash
sudo apt-get update
# Needed to use the driver
sudo apt-get install -y libmodbus-dev 
```

### Clone

```bash
cd ~/colcon_ws/src
git clone https://springlabsdevs.net/mecatronica/robotica/zlac8015d_driver2_cpp

```

---

## Usage

### Standalone Driver (Without ROS2)

**Compile the test application:**
```bash
cd ~/colcon_ws/src/zlac8015d_driver2_cpp
g++ -std=c++17 -Iinclude \
  src/zlac8015d_driver/zlac8015d_driver.cpp \
  src/zlac8015d_driver/test_driver.cpp \
  -o test_driver -lmodbus
```

**Run tests:**
```bash
./test_driver /dev/WHEELS
```

### ROS2 Node

**Build**
```bash
cd ~/colcon_ws
colcon build --packages-select zlac8015d_driver2_cpp --cmake-args -DCMAKE_DEBUG_FLAG=ON
source install/setup.bash
```

**NOTE:** If 'MAKE_DEBUG_FLAG' is not set, by default release mode will be used.


**Launch the test driver:**
```bash
# test_driver is also executable with ROS2 interface
ros2 run zlac8015d_driver2_cpp test_driver /dev/WHEELS
```

**Launch the node:**
```bash
ros2 run zlac8015d_driver2_cpp wheels_driver /dev/WHEELS
```

**Send velocity commands:**
```bash
ros2 topic pub /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.6}}"
```

**Monitor wheel data:**
```bash
ros2 topic echo /wheel/left_data
ros2 topic echo /wheel/right_data
```

---

### First-time setup: Speed resolution (0.1 RPM vs 1 RPM)

> [!WARNING]
> Some ZLAC8015D units **do not support changing speed resolution**.
> Perform this procedure with the robot **lifted off the ground** or with a safe test stand.
> The controller **may move** if the motors are enabled.

**How to check support (safe procedure):**
1. Ensure motors are **disabled** (use `emergency_stop()` or your hardware E-Stop).
2. Run the standalone test:
   ```bash
   ./test_driver /dev/ttyUSB0
   #or 
   ros2 run zlac8015d_driver2_cpp test_driver /dev/ttyUSB0
   ```
   The test will attempt to configure 0.1 RPM resolution.
3. **Power-cycle** the driver (turn OFF, wait ~10s, turn ON).
4. Run the test again:
    - If the resolution warning disappears → your unit supports **0.1 RPM.**
    - If the warning persists → your unit only supports **1 RPM** (keep `resolution_mode:=false`).  
    > If you switch between 0.1 and 1 RPM, command scaling changes by **×10**.


## Configuration

Edit ROS2 parameters in your launch file or via command line:

```bash
ros2 run zlac8015d_driver2_cpp wheels_driver /dev/WHEELS \
    --ros-args \
    -p wheel_radius:=0.1 \
    -p wheels_separation:=0.315 \
    -p accel_time_ms:=3000 \
    -p VelocityGains.left.kp:=60 

# You can set more values, please check the node as a reference
```
---

## Driver Notes

> [!IMPORTANT]
> Pay Attention to the next recommendations and utilities of the driver
> Check the Header File to get more information on a specific function

### Recommended startup sequence
1. Connect to `/dev/ttyUSBX`
2. Switch controller to **the desired mode** (**Velocity** recommended for `/cmd_vel_safe`)
3. Apply accel/decel time and gains
4. Validate speed resolution (and force safe mode if mismatch)
5. Enable command topics (`/cmd_vel_safe`)

### Safety
- Always send `0 RPM` before shutting down the node.
- On shutdown, the node attempts to stop the motors.
- Use an external E-Stop for real hardware testing.

## Stop Messages
- `emergency_stop()` — Immediately locks the wheels (no motion allowed).
- `disable_stop()` — Releases the lock so the wheels can move again.
- `enable_motor()` — Required after stopping the motors with either stop function.

### Common issues
- **Weird RPM readings (too high/too low):** resolution mismatch (0.1 vs 1 RPM).


---
