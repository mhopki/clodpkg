#!/bin/bash

# Launch roscore
roscore &
ros_pid=$!

# Wait for a few seconds to ensure roscore is up
sleep 5

# Set ROS_MASTER_URI to your computer's IP address and default port
export ROS_MASTER_URI=http://localhost:11311

# Launch the roslaunch file
roslaunch clod_pkg joy.launch &
roslaunch clod_pkg rviz.launch &

wait