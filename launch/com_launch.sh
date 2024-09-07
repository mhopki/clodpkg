#!/bin/bash

# Get your IP address
my_ip=$(hostname -I | awk '{print $1}')

# Define a variable based on your IP address
my_variable="default_value"

# Check your IP address and set the variable accordingly
if [[ "$my_ip" == "192.168.245.95" ]]; then
	#phone hotspot
	export ROS_IP=192.168.245.95 #172.56.29.230 #"192.168.245.95"
	export ROS_MASTER_URI=http://192.168.245.95:11311 #http://172.56.29.230:11311 #http://192.168.245.95:11311
elif [[ "$my_ip" == "192.168.129.249" ]]; then
	#mrsl_perch
	export ROS_IP=192.168.129.249 
	# Set ROS_MASTER_URI to your computer's IP address and default port
	export ROS_MASTER_URI=http://192.168.129.249:11311
elif [[ "$my_ip" == "192.168.1.100" ]]; then
	#scalar_av
	export ROS_IP=192.168.1.100 
	# Set ROS_MASTER_URI to your computer's IP address and default port
	export ROS_MASTER_URI=http://192.168.1.101:11311
elif [[ "$my_ip" == "192.168.8.60" ]]; then
	#mrsl_mast
	export ROS_IP=192.168.8.60
	# Set ROS_MASTER_URI to your computer's IP address and default port
	export ROS_MASTER_URI=http://192.168.8.60:11311
elif [[ "$my_ip" == "10.103.121.167" ]]; then
	#mrsl_mast
	export ROS_IP=10.103.121.167
	# Set ROS_MASTER_URI to your computer's IP address and default port
	#export ROS_MASTER_URI=http://10.103.121.167:11311
	export ROS_MASTER_URI=http://10.103.90.126:11311
elif [[ "$my_ip" == "192.168.50.204" ]]; then
    #mrsl_mast
    export ROS_IP=192.168.50.204
    # Set ROS_MASTER_URI to your computer's IP address and default port
    #export ROS_MASTER_URI=http://10.103.121.167:11311
    export ROS_MASTER_URI=http://192.168.50.205:11311
elif [[ "$my_ip" == "192.168.80.95" ]]; then
    #mrsl_mast
    export ROS_IP=192.168.80.95
    export ROS_MASTER_URI=http://192.168.80.171:11311
    # Set ROS_MASTER_URI to your computer's IP address and default port
    #export ROS_MASTER_URI=http://10.103.121.167:11311
    export ROS_MASTER_URI=http://192.168.80.171:11311
fi

# Launch roscore
#roscore &
#ros_pid=$!

# Wait for a few seconds to ensure roscore is up
#sleep 5

#export ROS_IP=192.168.8.60
# Set ROS_MASTER_URI to your computer's IP address and default port
#export ROS_MASTER_URI=http://192.168.8.60:11311

# Launch the roslaunch file
roslaunch clod_pkg joy.launch &
#roslaunch clod_pkg rviz.launch &

wait
