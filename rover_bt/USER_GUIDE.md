# Rover Behavior Tree — User Guide

Welcome to the `rover_bt` user guide. This document explains how to interact with the autonomous rover, use voice commands, operate it manually, and understand what the rover is doing at any given time.

## 1. High-Level Overview

The rover is controlled by a central "brain" called a Behavior Tree. You can think of it as a state machine that decides what the rover should be doing based on your commands, sensor data, and safety checks.

### Operating Modes
At any moment, the rover is in one of the following modes:
* **IDLE**: Standing by, ready to receive a new command (e.g., waiting to navigate).
* **AUTONOMOUS**: Driving to a designated location (waypoint) while avoiding obstacles.
* **PATROL**: Endlessly looping through a predefined sequence of waypoints.
* **TELEOP_VOICE**: Waiting for short, directional voice commands (e.g., "avanza", "retrocede").
* **TELEOP_JOYCON**: Under manual control via the joystick controller.
* **PERSON_TRACK**: Following a designated person visually.
* **EMERGENCY**: Highest priority mode. All normal commands are ignored until the emergency is resolved and a "resume" command is given.

*Note: Mapping (`start_mapping` / `stop_mapping`) runs as an "overlay" and can happen while the rover is in IDLE or TELEOP.*

---

## 2. Commanding the Rover

There are several ways to command the rover:
1. **Voice Commands** (using the wake word)
2. **Joystick** (Joycon)
3. **Command Line / GUI** (ROS 2 service calls)

### 2.1 Voice Commands

The rover continuously listens for its wake word (by default: `comando` or `ok robot`, depending on your configuration). Once it hears the wake word, it will beep and listen for a command.

> 🚨 **Emergency Exception:** You can shout **`emergencia`** at any time. It bypasses the wake word and immediately stops the rover.

#### Navigation & Movement
| You Say (Spanish) | What it Does |
|---|---|
| `ve a <lugar>` or `ir a <lugar>` | Navigates autonomously to the known location `<lugar>` (e.g., "ve a la cocina"). |
| `patrulla` | Starts an endless patrol route visiting all configured waypoints. |
| `detente`, `alto`, or `para` | Stops the rover and cancels the current action (navigation, teleop, etc.). |

#### Voice Teleoperation
First, say `modo voz` or `teleop voz`. The rover will confirm. Then you can use short movement commands:
| You Say | What it Does |
|---|---|
| `avanza` or `adelante` | Drives forward briefly. |
| `retrocede` or `atrás` | Drives backward briefly. |
| `izquierda` | Turns left in place. |
| `derecha` | Turns right in place. |
| `modo autónomo` | Exits Voice Teleop and returns to IDLE (ready for navigation). |

#### Follow Me & Mapping
| You Say | What it Does |
|---|---|
| `seguir persona`, `sígueme` | Enters Person Tracking mode. The rover will try to follow you. |
| `inicia mapeo` | Starts creating a brand new map of the environment. |
| `modo slam`, `mejora el mapa` | Continues mapping on top of the existing map (SLAM). |
| `detén el mapeo`, `guarda el mapa`| Stops mapping and saves the map to disk. |

#### Saving Locations & Status
| You Say | What it Does |
|---|---|
| `guarda este lugar como <nombre>` | Saves the rover's current physical location as `<nombre>`. You can now say "ve a `<nombre>`". |
| `estado` or `dónde estás` | The rover will announce its current mode and distance to the goal. |
| `emergencia` | Immediately stops all movement and enters EMERGENCY mode. |
| `reanuda` | Leaves EMERGENCY mode and returns to IDLE. |

### 2.2 Joycon (Joystick) Control

You can take manual control of the rover at any time using the joystick. This is automatically detected by the system (Joycon Overlay).

1. Hold the **deadman switch** (typically L1 or R1).
2. Move the joystick.
3. The rover will instantly switch to **TELEOP_JOYCON** mode and respond to your inputs.
4. When you release the deadman switch, the rover will automatically return to **IDLE**.
5. *Note: If you press the `Y` button, the rover will also request to switch to AUTONOMOUS readiness.*

### 2.3 Command Line (Service Calls)

If you are a developer or using a GUI dashboard, commands can be sent directly to the `/rover_bt/send_command` service:

```bash
# Navigate to the kitchen
ros2 service call /rover_bt/send_command rover_bt/srv/SendCommand "{source: 'gui', command: 'navigate', target: 'cocina'}"

# Stop the rover
ros2 service call /rover_bt/send_command rover_bt/srv/SendCommand "{source: 'gui', command: 'stop', target: ''}"

# Start person tracking
ros2 service call /rover_bt/send_command rover_bt/srv/SendCommand "{source: 'gui', command: 'person_track', target: ''}"
```

---

## 3. Monitoring the Rover's Status

You can see exactly what the rover is thinking and doing by reading its status topic. The brain publishes a summary 1 time per second.

Run this in your terminal:
```bash
ros2 topic echo /rover_bt/status
```

**Key fields to watch:**
* `mode`: The current state (IDLE, AUTONOMOUS, EMERGENCY, etc.).
* `navigation_status`: What the planner is doing (`navigating`, `succeeded`, `failed`, `stalled`).
* `goal_distance`: Distance to the target in meters (-1 if no goal).
* `sensor_health`: Shows if the LiDAR, camera, motors, or odometry have disconnected or stopped sending data.

## 4. Recovering from an Emergency

If the rover stops for an emergency, it will enter **EMERGENCY** mode. 

* **Commanded Emergency:** If you triggered it via voice or button, you must explicitly say **`reanuda`** (resume) or send the `resume` command to continue.
* **Odometry/Sensor Loss:** If the emergency was triggered automatically because a critical sensor (like odometry or wheel encoders) disconnected, the rover will announce the failure. Once the sensor comes back online, the rover will **automatically recover** and return to IDLE.
