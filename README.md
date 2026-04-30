# autonomous_rover

## HACER MAPA



sudo apt update
sudo apt install ros-jazzy-rtabmap-ros ros-jazzy-imu-filter-madgwick ros-jazzy-pcl-ros


ros2 run azure_kinect_ros2_driver azure_kinect_node --ros-args -p point_cloud:=true -p rgb_point_cloud:=true


ros2 run imu_filter_madgwick imu_filter_madgwick_node   --ros-args   -p use_mag:=false   -p world_frame:=enu   -p publish_tf:=false   -r imu/data_raw:=/k4a/imu   -r imu/data:=/k4a/imu_filtered


ros2 launch rover_bringup hardware_bringup.launch.py 

ros2 launch rtabmap_launch rtabmap.launch.py   rtabmap_args:="--delete_db_on_start \
  --Reg/Force3DoF true \
  --Grid/FromDepth false \
  --Grid/3D false \
  --Grid/RangeMax 4.5 \
  --Grid/MaxGroundHeight 0.10 \
  --Grid/MaxObstacleHeight 1.20 \
  --Grid/CellSize 0.05"   rgb_topic:=/k4a/rgb/image_raw   depth_topic:=/k4a/depth_to_rgb/image_raw   camera_info_topic:=/k4a/rgb/camera_info   scan_cloud_topic:=/k4a/points2   subscribe_scan_cloud:=true   imu_topic:=/k4a/imu_filtered   wait_imu_to_init:=true   frame_id:=base_link   approx_sync:=true   approx_sync_max_interval:=0.02   wait_for_transform:=0.3   queue_size:=20   qos:=2   rviz:=true

ros2 run robot_core teleop_keyboard.py --ros-args -r cmd_vel:=/cmd_vel_safe
