# Health status for a single sensor
string name           # lidar, camera, odom, imu, motor
uint8 status          # 0=OK, 1=WARNING, 2=ERROR, 3=STALE
float32 last_seen_ago # seconds since last message
string detail         # human-readable detail

uint8 OK      = 0
uint8 WARNING = 1
uint8 ERROR   = 2
uint8 STALE   = 3
