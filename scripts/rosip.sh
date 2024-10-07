#!/bin/bash

my_ip=$(hostname -I | awk '{print $1}')


#ROBOT
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
elif [[ "$my_ip" == "10.0.65.10" ]]; then
        # Set ROS_MASTER_URI
        export ROS_IP=10.103.90.126
        #export ROS_MASTER_URI=http://10.103.121.167:11311
        export ROS_MASTER_URI=http://10.103.90.126:11311
elif [[ "$my_ip" == "192.168.50.205" ]]; then
        # Set ROS_MASTER_URI
        export ROS_IP=192.168.50.205
        #export ROS_MASTER_URI=http://10.103.121.167:11311
        export ROS_MASTER_URI=http://192.168.50.205:11311
elif [[ "$my_ip" == "192.168.80.171" ]]; then
        # Set ROS_MASTER_URI
        export ROS_IP=192.168.80.171
        #export ROS_MASTER_URI=http://10.103.121.167:11311
        export ROS_MASTER_URI=http://192.168.80.171:11311
fi



# COMPUTER
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
elif [[ "$my_ip" == "10.103.114.83" ]]; then
    #mrsl_mast
    export ROS_IP=10.103.114.83
    # export ROS_MASTER_URI=http://192.168.80.171:11311
    # Set ROS_MASTER_URI to your computer's IP address and default port
    export ROS_MASTER_URI=http://10.103.121.167:11311
    # export ROS_MASTER_URI=http:i//192.168.80.171:11311
elif [[ "$my_ip" == "10.103.64.139" ]]; then
    #mrsl_mast
    export ROS_IP=10.103.64.139
    # export ROS_MASTER_URI=http://192.168.80.171:11311
    # Set ROS_MASTER_URI to your computer's IP address and default port
    export ROS_MASTER_URI=http://10.103.64.139:11311
    # export ROS_MASTER_URI=http:i//192.168.80.171:11311
fi

echo $my_ip
echo $ROS_IP
echo $ROS_MASTER_URI

wait
