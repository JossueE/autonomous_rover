






++++++++++++++++++++++++++++++++++++++++++++++++++++++++    # odom_comparison

Independent ROS 2 benchmark package for comparing encoder-only wheel odometry
against RTAB-Map odometry on the rover.

## Objective

The experiment evaluates two odometry sources under the same motion commands:

| Source | Topic | Role |
| --- | --- | --- |
| Wheel odom without IMU | `/wheel/odom` | Encoder-only differential odometry from `odometry2` |
| RTAB-Map odom | `/rtabmap/odom` | RGB-D/ICP odometry from Azure Kinect + RTAB-Map |

Only RTAB-Map publishes the dynamic TF `odom -> base_footprint`. The wheel
odometry node is launched with `publish_tf:=false`, so both odometry topics can
coexist without competing for the same TF edge. The EKF and `odometry2/imu` are
not launched; wheel odom is intentionally encoder-only.

## Build

```bash
cd ~/colcon_ws
colcon build --packages-select odom_comparison
source install/setup.bash
```

## Bringup

Do not run Nav2, BT, teleop, or any other node that publishes rover velocity
commands during these tests.

```bash
ros2 launch odom_comparison benchmark.launch.py robot:=zlac706
```

The launcher starts:

- `robot_state_publisher`
- ZLAC wheels driver
- `twist_priority_mux`, using `/cmd_vel_test` as the benchmark command input
- `odometry2` with TF disabled
- Azure Kinect + Madgwick + RTAB-Map through the existing rover launch
- `odom_compare_recorder`

## Run Trials

List available trials:

```bash
ros2 run odom_comparison trial_runner -- --list
```

Run the 2 m straight-line test:

```bash
ros2 run odom_comparison trial_runner -- --trial straight_2m
```

Run all paper-style tests individually:

```bash
ros2 run odom_comparison trial_runner -- --trial straight_2m
ros2 run odom_comparison trial_runner -- --trial yaw_sweep_90
ros2 run odom_comparison trial_runner -- --trial free_run
```

The `free_run` trial records until you press `Ctrl+C`. It does not publish
automatic velocity commands; drive the rover manually through `/cmd_vel_test`
while the recorder is running.

Each run writes results to:

```text
odom_comparison/images/<run_id>/
```

## Physical References

Use floor marks and a tape measure before starting each run.

| Trial | Physical reference |
| --- | --- |
| `straight_2m` | End mark 2.0 m forward from start |
| `yaw_sweep_90` | Same position; final heading returns to 0 deg after left 90, center, right 90, center |
| `free_run` | Operator-defined; update `config/trials.yaml` reference if using a measured final mark |

Keep speeds low and stop the robot if the physical path diverges from the test
area. If the robot does not reach the physical reference exactly, record that in
the run notes and use the final physical mark as the reference in
`config/trials.yaml`.

## Outputs

Each trial directory contains:

- `raw_samples.csv`: raw and relative pose samples for both odometry sources
- `rtabmap_info.csv`: RTAB-Map odometry health data
- `summary_metrics.csv`: final error, path length, rate, disagreement, lost rate
- `metrics_table.png`: compact table for the report
- `trajectory_xy.png`: relative XY trajectory comparison
- `yaw_error.png`: relative yaw response
- `README_results.md`: automatic result summary and interpretation template

## Table Justification

The final-error table evaluates accuracy against the physical marks. The static
table evaluates drift and noise when the true motion is zero. The RTAB-Map lost
percentage evaluates robustness, because visual/ICP odometry can temporarily
lose tracking. The trajectory and yaw plots make systematic effects visible:
wheel slip, yaw bias, lateral drift, visual odometry resets, and closure error.

## Analysis Guidance

Wheel odometry should be strong on short straight segments when wheel radius and
track width are calibrated, but it accumulates heading and closure error through
slip and small geometric biases. RTAB-Map can reduce accumulated drift when RGB-D
features and ICP geometry are well constrained, but it can degrade with poor
lighting, low texture, sparse geometry, motion blur, or fast turns.

Do not claim continuous global accuracy from these tests. The floor marks provide
final-state and closure references, not a continuous ground-truth trajectory.
