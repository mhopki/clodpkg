#!/bin/bash

my_ip=$(hostname -I | awk '{print $1}')

if [[ "$my_ip" == "192.168.1.101" ]]; then
	# Set ROS_MASTER_URI
	export ROS_IP=192.168.1.101
	export ROS_MASTER_URI=http://192.168.1.100:11311
elif [[ "$my_ip" == "192.168.129.207" ]]; then
	# Set ROS_MASTER_URI
	export ROS_IP=192.168.129.207
	export ROS_MASTER_URI=http://192.168.129.249:11311
fi

#mrsl_mast
#export ROS_IP=192.168.8.56
# Set ROS_MASTER_URI
#export ROS_MASTER_URI=http://192.168.8.60:11311

#def
#export ROS_IP=192.168.1.178
# Set ROS_MASTER_URI
#export ROS_MASTER_URI=http://192.168.1.154:11311

# Launch the roslaunch file
roslaunch clod_pkg clod_only.launch &
#roslaunch pointgrey_camera_driver camera.launch &
roslaunch spinnaker_camera_driver camera.launch &
roslaunch realsense2_camera rs_t265.launch & 
roslaunch ublox_gps ublox_device_beast.launch param_file_name:=alice param_file_dir:=/home/beast/catkin_ws/src/ublox/ublox_gps/config &
roslaunch ntrip_client ntrip_client.launch 


#roslaunch clod_pkg clod.launch &


wait