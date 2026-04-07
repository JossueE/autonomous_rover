# odometry2

This package is used to compute the odometry of a differential drive mobile robot using a custom class (ENCODER).

The ENCODER class is created as a normal C++ class in order to avoid referencing any ROS dependency. After that,
the 'odometry2' node creates an instance of that class to use it.

For more details, please refer to 'src/odometry2.cpp' file.
