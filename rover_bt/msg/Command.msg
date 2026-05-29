# A typed command from any input source
builtin_interfaces/Time stamp
string source        # voice, joycon, gui, ros_service, emergency
uint8 priority       # 0=highest (emergency), 4=lowest (ros_service)
string command       # navigate, stop, patrol, teleop_voice, etc.
string target        # waypoint name (for navigate), empty otherwise

uint8 PRIORITY_EMERGENCY   = 0
uint8 PRIORITY_JOYCON      = 1
uint8 PRIORITY_VOICE       = 2
uint8 PRIORITY_GUI         = 3
uint8 PRIORITY_ROS_SERVICE = 4
