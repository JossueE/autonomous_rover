# BLUESEA ROS2 driver #

## Overview ##
----------
BLUESEA ROS2 driver is specially designed to connect to the lidar by robotbase2. The driver can run on operating systems with ROS2 Humble installed, and mainly supports ubuntu 22.04 LTS. 

## Get and build the PACECAT ROS driver package ##
1.Get the PACECAT ROS2 driver from Gitlab and deploy the corresponding location ~/colcon_ws/src

    git clone git@springlabsdevs.net:mecatronica/robotica/pacecat_bluesea_ros2.git  
2.Parameters configuration, if your lidar model is LDS-50C-R, change yaml file.

    inverted: true
    reversed: false
    baud_rate: 921600

3.Build

    colcon build --packages-select pacecat
3.Update the current ROS2 package environment

    source ./install/setup.sh


4.Using ROS2 launch to run drivers

	sudo chmod 777 /dev/ttyUSB0 	// If it is a serial/virtual serial port model, it needs to be authorized
    
    ros2 launch pacecat uart_lidar.launch.py  



## Autor
**M. Garcia**

- :Website: [octopy.com](https://octopy.com/)
- :Facebook: [OctopyTech](https://www.facebook.com/OctopyTech/)
- :LinkedIn: [LinkedIn_Octopy](https://mx.linkedin.com/company/octopytech)

