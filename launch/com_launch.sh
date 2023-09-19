#!/bin/bash

#mrsl_perch
#export ROS_IP=192.168.129.249 
# Set ROS_MASTER_URI to your computer's IP address and default port
#export ROS_MASTER_URI=http://192.168.129.249:11311

#scalar_av
export ROS_IP=192.168.1.100 
# Set ROS_MASTER_URI to your computer's IP address and default port
export ROS_MASTER_URI=http://192.168.1.100:11311


#mrsl_mast
#export ROS_IP=192.168.8.60
# Set ROS_MASTER_URI to your computer's IP address and default port
#export ROS_MASTER_URI=http://192.168.8.60:11311


# Launch roscore
roscore &
ros_pid=$!

# Wait for a few seconds to ensure roscore is up
sleep 5

#export ROS_IP=192.168.8.60
# Set ROS_MASTER_URI to your computer's IP address and default port
#export ROS_MASTER_URI=http://192.168.8.60:11311

# Launch the roslaunch file
roslaunch clod_pkg joy.launch &
roslaunch clod_pkg rviz.launch &

wait
