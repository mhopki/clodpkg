#!/bin/bash

# Set ROS_MASTER_URI
export ROS_MASTER_URI=http://192.168.1.154:11311

# Launch the roslaunch file
roslaunch clod_pkg clod.launch
