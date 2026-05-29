# Overall robot status published at 1Hz on /rover_bt/status
builtin_interfaces/Time stamp
string mode                    # IDLE, AUTONOMOUS, PATROL, etc.
string active_command_source   # voice, joycon, gui, ros_service
string navigation_status       # idle, navigating, recovering, failed
float32 goal_distance          # meters to goal (-1 if no goal)
string target_location         # name of target location
rover_bt/SensorHealth[] sensor_health  # per-sensor health
bool is_mapping                # RTAB-Map mapping active
string mapping_mode            # mapping, slam, localization, off
