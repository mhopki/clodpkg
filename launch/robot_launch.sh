#!/bin/bash

#mrsl_perch
export ROS_IP=192.168.129.207
# Set ROS_MASTER_URI
export ROS_MASTER_URI=http://192.168.129.249:11311


#mrsl_mast
#export ROS_IP=192.168.8.56
# Set ROS_MASTER_URI
#export ROS_MASTER_URI=http://192.168.8.60:11311

#def
#export ROS_IP=192.168.1.178
# Set ROS_MASTER_URI
#export ROS_MASTER_URI=http://192.168.1.154:11311



# Launch the roslaunch file
roslaunch clod_pkg clod.launch &
roslaunch realsense2_camera rs_t265.launch &

wait
