#!/bin/bash

# Launch roscore
#roscore &
#ros_pid=$!

# Wait for a few seconds to ensure roscore is up
#sleep 5

#export ROS_IP=192.168.8.60
# Set ROS_MASTER_URI to your computer's IP address and default port
#export ROS_MASTER_URI=http://192.168.8.60:11311

# Launch the roslaunch file
roslaunch clod_pkg o_track_multi.launch r_num:=1 &
roslaunch clod_pkg o_track_multi.launch r_num:=2 &
roslaunch clod_pkg o_track_multi.launch r_num:=3 &
roslaunch clod_pkg o_track_multi.launch r_num:=4 &
#roslaunch clod_pkg rviz.launch &

wait
