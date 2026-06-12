# teleop

Keyboard and Joy-Con teleoperation package for a differential-drive rover.

The package is `ament_python` and installs two console scripts.

## Executables

| Executable | Purpose | Topics |
| --- | --- | --- |
| `teleop_keyboard` | Terminal keyboard velocity control | Publishes `cmd_vel_safe` |
| `teleop_joycon` | Game controller velocity control with deadman switch | Subscribes `joy`, publishes `cmd_vel_safe` |

## Keyboard Teleop

```bash
ros2 run teleop teleop_keyboard
```

Controls from the script:

- `w` / `x`: increase or decrease linear velocity.
- `a` / `d`: increase or decrease angular velocity.
- `p`: set max linear velocity.
- `space` or `s`: force stop.
- `Ctrl+C`: quit.

The script publishes a final zero Twist on shutdown.

## Joy-Con Teleop

Start a joystick driver first:

```bash
ros2 run joy joy_node
```

Then run:

```bash
ros2 run teleop teleop_joycon
```

The node uses the left joystick for linear/angular motion and requires the RT trigger deadman switch to be held before it publishes motion commands.

## Parameters

`teleop_joycon` declares:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `max_lin_vel` | `0.7` | Maximum linear velocity scale. |
| `max_ang_vel` | `0.8` | Maximum angular velocity scale. |
| `axis_linear` | `1` | Joy axis for linear motion. |
| `axis_angular` | `0` | Joy axis for angular motion. |

In `rover_bringup/hardware_bringup.launch.py`, `max_lin_vel` and `max_ang_vel` are overridden from `rover_bringup/config/robot_profiles.yaml`.
