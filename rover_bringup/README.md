# rover_bringup

Real rover launch orchestration for the autonomous rover.

This package installs the launch files, robot profiles, maps, RViz views, and helper scripts used to start the physical robot stack.

## Main Commands

Full stack:

```bash
ros2 launch rover_bringup bringup_all.launch.py
```

Full stack through the tmux TUI wrapper:

```bash
ros2 run rover_bringup bringup_all_tui.sh --restart
```

Hardware only:

```bash
ros2 launch rover_bringup hardware_bringup.launch.py robot:=zlac706
ros2 launch rover_bringup hardware_bringup.launch.py robot:=zlac8015d motors_port:=/dev/ttyUSB0
```

Azure Kinect plus RTAB-Map:

```bash
ros2 launch rover_bringup k4a_rtabmap.launch.py mode:=localization
ros2 launch rover_bringup k4a_rtabmap.launch.py mode:=mapping
ros2 launch rover_bringup k4a_rtabmap.launch.py mode:=slam
```

Nav2 stack:

```bash
ros2 launch rover_bringup nav2_navigation.launch.py
```

## Robot Profiles

`config/robot_profiles.yaml` defines the supported robot profiles:

- `zlac8015d`: uses `zlac8015d_driver2_cpp` and a single `motors_port`.
- `zlac706`: uses `zlac706_driver2_cpp` and separate `motor_left_port` / `motor_right_port`.

The profile also supplies wheel geometry, velocity limits, camera placement, map path, point cloud ROI values, and NMPC geometry overrides.

## Launch Files

| Launch file | Purpose | Important arguments |
| --- | --- | --- |
| `bringup_all.launch.py` | Full real rover stack: hardware, RTAB-Map, planner, NMPC, BT | `robot`, `rtabmap_mode`, `log_level` |
| `hardware_bringup.launch.py` | Motor driver, robot state publisher, joystick, Joy-Con teleop, twist mux | `robot`, `motors_port`, `motor_left_port`, `motor_right_port`, `log_level` |
| `k4a_rtabmap.launch.py` | Azure Kinect, Madgwick IMU filter, RTAB-Map | `mode`, `rtabmap_args`, `log_level` |
| `nav2_navigation.launch.py` | Nav2 navigation servers and optional RViz | `use_sim_time`, `params_file`, `sensor_topic`, `map_topic`, `odom_topic`, `cmd_vel_topic`, `rviz` |

## Scripts

- `scripts/bringup_all_tui.sh`: starts the full bringup inside a tmux session and separates voice logs from other logs.
- `scripts/twist_priority_mux.py`: priority Twist multiplexer.

Twist priority order in code:

1. `/cmd_vel_safe`
2. `/cmd_vel_person`
3. `/cmd_vel_nav`

The selected command is published to `/cmd_vel`.

## Configuration

- `config/robot_profiles.yaml`: robot-specific hardware and geometry.
- `config/ekf.yaml`: hardware EKF configuration.
- `maps/`: Lanelet/OSM maps used by planner profiles.
- `rviz/nav.rviz`: real navigation RViz view.
- `rviz/sim.rviz`: simulation RViz view.

## Notes

`bringup_all.launch.py` launches subsystems in staggered order so localization/odometry is available before planner, NMPC, and behavior tree watchdogs start.
