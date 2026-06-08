# Rover Behavior Tree — Technical Guide

The `rover_bt` package provides the central Behavior Tree (BT) brain for the autonomous rover, utilizing [BehaviorTree.CPP v4](https://www.behaviortree.dev/). This node manages mode transitions, navigation supervision, command arbitration, sensor watchdogs, and spoken (TTS) feedback.

## 1. Build & Install

The node builds with `colcon`. First, install the necessary voice/TTS models:

```bash
# From the workspace src/ directory
bash rover_bt/scripts/install_voice_assets.sh
```

Then build the package:
```bash
colcon build --packages-select rover_bt --symlink-install
source install/setup.bash
```

## 2. Launching

**Real Hardware:**
```bash
ros2 launch rover_bt rover_bt.launch.py
```

**Simulation (Gazebo):**
```bash
ros2 launch rover_bt rover_bt_sim.launch.py
```
*Note: In simulation, `use_sim_time` is strictly required to prevent instantaneous watchdog timeouts.*

## 3. Behavior Tree Architecture

The tree evaluates at 10 Hz. The root is a **`ReactiveSequence`**, ensuring that high-priority checks (command processing, safety, joycon) are evaluated on every single tick, even if a later branch (like navigation) is currently `RUNNING`.

### Layers:
1. **Layer 0 (`ProcessCommand`)**: Consumes the highest-priority command from the arbitrator and pushes it to the blackboard. This runs before Safety so emergency commands trigger in the exact same tick.
2. **Layer 1 (`SafetyLayer`)**: Checks for emergency commands, wheel motor power loss, and odometry (`/rtabmap/odom_info_lite`) staleness. Automatically forces `EMERGENCY` mode on failure.
3. **Layer 2 (`SystemHealthMonitor`)**: Parallel watchdogs for LiDAR, camera, IMU, and motors. These warn but don't strictly e-stop unless it's a critical component caught in Layer 1.
4. **Layer 4 (`MappingOverlay`)**: Handles `start_mapping`, `start_slam`, and `stop_mapping`. This is orthogonal to the drive modes.
5. **Layer 4.5 (`JoyconOverlay`)**: Uses `CheckJoyActive` to detect manual deadman/joystick input. If detected, forcibly sets `TELEOP_JOYCON` mode.
6. **Layer 5 (`ModeDispatch`)**: A `Switch7` node on the `{mode}` blackboard variable. Branches into the respective subtrees (EMERGENCY, TELEOP_VOICE, TELEOP_JOYCON, AUTONOMOUS, PATROL, IDLE, PERSON_TRACK).

## 4. Subtrees and Mode Logic

The state machine is driven by the `{mode}` variable.

* **Autonomous/Patrol Subtrees**: Run `NavigateToGoal` as the default leaf in a `ReactiveFallback`. The supervisor nodes (`CheckNavStatus` for `failed`, `stalled`, `planner_timeout`, `succeeded`) continuously monitor the action client and can pre-empt the drive. Tiered recovery tries to request a replan before giving up.
* **PersonTrack Subtree**: Asserts `/person_tracker/enable`. Can live-switch indoor/outdoor profiles (using `SetPersonProfile`) without leaving the mode. Uses `CheckPersonEvent` to monitor tracking state and announce if the target is lost or found.
* **Emergency Mode**: "Sticky". It rejects all commands except `resume`. It features auto-resume capabilities: if triggered by an odometry loss or wheel disconnect, it will automatically transition back to `IDLE` once the sensor signal is restored.

## 5. Command Arbitration

All incoming commands (Voice, Joycon, GUI, Services) route through `CommandArbitrator` (priority queue).
0. `emergency_stop` (highest)
1. `joycon`
2. `voice`
3. `gui`
4. other

## 6. The Blackboard Contract

The tree heavily relies on `_autoremap="true"` in subtrees to share root blackboard variables:
* `context`: C++ `SharedContext` for pose, sensor times.
* `mode`: Master FSM state.
* `command`, `active_command_source`, `target_location`: Command payload.
* `nav_status`, `goal_distance`, `recovery_attempted`: Navigation feedback.
* `is_mapping`, `mapping_mode`, `patrol_index`: Overlays and sequences.

## 7. Custom BT Node Catalog

Registered in `src/rover_bt_node.cpp`.

**Conditions:**
* `CheckMode`, `CheckCommand`, `CheckNavStatus`, `CheckFlag`
* `CheckSensorHealth`: Validates timestamp freshness against `now()`.
* `CheckJoyActive`: Validates deadman switch on joypad.
* `CheckPersonEvent`: Monitors YOLO/OSNet tracker status edges.

**Actions:**
* `ProcessCommand`, `ClearCommand`
* `NavigateToGoal`: Action client wrap for driving.
* `MoveRover`, `ZeroTwist`: Open-loop velocity control.
* `SetMode`, `SetFlag`
* `Speak`: Piper TTS interface.
* `StartMapping`, `StopMapping`
* `RequestReplan`
* `IncrementPatrolIndex`
* `SaveLocation`: Uses `LocationRegistry` to dump the current pose.
* `SetPersonProfile`: Configures `/person_tracker/profile`.
* `PublishStatus`: Dumps state to `/rover_bt/status`.

## 8. Interfaces (Topics, Services, Actions)

**Topics:**
* Publishes: `/rover_bt/status` (1 Hz), `/cmd_vel_safe`.
* Subscribes: `/rover_bt/commands`, `/rtabmap/odom`, `/rtabmap/odom_info_lite`, `/amcl_robot_pose`, `/scan`, `/k4a/points2`, `/imu/data`, `/sdv_trajectory`, `/joy`.

**Services:**
* `/rover_bt/send_command` (`rover_bt/srv/SendCommand`)
* `/rover_bt/save_location` (`rover_bt/srv/SaveLocation`)

**Actions:**
* Client to `navigate_to_goal` (`rover_bt/action/NavigateToGoal`).

## 9. Extending the BT

1. **Write the Node**: Inherit from `BT::ConditionNode` or `BT::SyncActionNode`. Place it in `include/...` and `src/nodes/...`.
2. **Register**: Add it to `init_behavior_tree()` in `rover_bt_node.cpp`.
3. **Use in XML**: Edit `trees/rover_bt_main.xml`. Thanks to `--symlink-install`, you do not need to rebuild for XML-only changes.
