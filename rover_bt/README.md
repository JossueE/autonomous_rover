# rover_bt

The central **behavior tree (BT)** brain for the autonomous rover. A single C++ node (`rover_bt_node`) ticks a [BehaviorTree.CPP v4](https://www.behaviortree.dev/) tree that owns the rover's high-level behaviour: mode management, navigation supervision, sensor-health watchdogs / emergency-stop, multi-input command arbitration (voice / joycon / GUI / service), mapping lifecycle, and spoken (TTS) feedback. A companion Python node (`voice_command_node.py`) does Spanish speech recognition and feeds commands in.

## Documentation

To better serve different audiences, our documentation is split into two specialized guides:

1. 📘 **[User Guide](USER_GUIDE.md)**: Intended for operators and end-users. Read this to learn how to interact with the rover using voice commands, the joycon, or the dashboard, and to understand the rover's various modes and behaviors.
2. 🛠️ **[Technical Guide](TECHNICAL_GUIDE.md)**: Intended for robotics developers. Read this to understand the behavior tree architecture, layers, custom C++ BT nodes, ROS 2 interfaces, and the internal blackboard contract.

---
