#!/bin/bash

my_ip=$(hostname -I | awk '{print $1}')

if [[ "$my_ip" == "192.168.245.96" ]]; then
	# Set ROS_MASTER_URI
	export ROS_IP=192.168.245.96 #172.56.223.168 #192.168.245.96
	export ROS_MASTER_URI=http://192.168.245.96:11311 #http://172.56.29.230:11311
elif [[ "$my_ip" == "192.168.1.101" ]]; then
	# Set ROS_MASTER_URI
	export ROS_IP=192.168.1.101
	export ROS_MASTER_URI=http://192.168.1.101:11311
elif [[ "$my_ip" == "192.168.129.207" ]]; then
	# Set ROS_MASTER_URI
	export ROS_IP=192.168.129.207
	export ROS_MASTER_URI=http://192.168.129.249:11311
elif [[ "$my_ip" == "10.103.90.126" ]]; then
	# Set ROS_MASTER_URI
	export ROS_IP=10.103.90.126
	#export ROS_MASTER_URI=http://10.103.121.167:11311
	export ROS_MASTER_URI=http://10.103.90.126:11311
elif [[ "$my_ip" == "192.168.50.205" ]]; then
        # Set ROS_MASTER_URI
        export ROS_IP=192.168.50.205
        #export ROS_MASTER_URI=http://10.103.121.167:11311
        export ROS_MASTER_URI=http://192.168.50.205:11311
fi

echo $ROS_IP
echo $ROS_MASTER_URI

wait
