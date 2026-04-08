sudo apt update
sudo apt install -y build-essential cmake gfortran git pkg-config liblapack-dev

cd /tmp/casadi
rm -rf build

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DWITH_IPOPT=ON \
  -DWITH_BUILD_IPOPT=ON \
  -DWITH_BUILD_MUMPS=ON \
  -DWITH_BUILD_REQUIRED=ON

cmake --build build -j"$(nproc)"
sudo cmake --install build

echo "/usr/local/lib" | sudo tee /etc/ld.so.conf.d/casadi.conf
sudo ldconfig



colcon build --packages-select nmpc_controller --cmake-args   -DCASADI_INCLUDE_DIR=/usr/local/include   -DCASADI_LIBRARY=/usr/local/lib/libcasadi.so

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

ros2 launch turtlebot3_bringup rviz2.launch.py

ros2 run nmpc_controller path_drawer.py

ros2 run nmpc_controller nmpc_controller_node --ros-args   -p map_frame:=odom   -p base_frame:=base_footprint   -p cmd_vel_topic:=/cmd_vel   -p costmap_topic:=/move_base/local_costmap/costmap   -p path_topic:=/drawn_plan

