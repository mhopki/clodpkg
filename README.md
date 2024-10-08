# clod_pkg

Package for controlling the BEAST robot platform

>Branches To Use:
>>Computer: new-com
>>Robot: new-rob

How to use TMUX:

>Computer:
>>run ./tmux_com.sh
>>Nothing runs on its own, but it sets up the prompts for you. Navigate to the desired tab and press enter to run the already written prompt.

>Robot:
>>run ./tmux_robot.sh
>>Everything runs on its own, Navigate to the desired tab to kill what you dont want running






INSTALL ON ROBOT:

>Install spinnaker sdk:

>>https://www.flir.com/support-center/iis/machine-vision/application-note/using-spinnaker-on-arm-and-embedded-systems/

>>(for Ubuntu 18.04 download Spinnaker 2.7.0.128)


>ROS-Camera-Driver:
>>Install flir_camera_driver:

>>>https://github.com/ros-drivers/flir_camera_driver

>>for Ubuntu 18.04: Install pointgrey_camera_driver (main-branch) OR for Ubuntu 20.04 use noetic-branch for 
>>>http://wiki.ros.org/pointgrey_camera_driver


>Flir/Pointgrey requirements:

>>sudo apt install ros-melodic-image-exposure-msgs 

>>sudo apt install ros-melodic-wfov-camera-msgs

>Ubuntu 18.04:
>>pip3 install rospkg

>Install adafruit_servokit:

>>Ubuntu 18.04:
>>https://learn.adafruit.com/circuitpython-libraries-on-linux-and-the-nvidia-jetson-nano/initial-setup
>>
>>python -m pip install --upgrade setuptools
python -m pip install --upgrade pip
>>
>>sudo apt-get update
sudo apt-get install libcups2-dev
>>sudo apt install libgirepository1.0-dev
>>

>Ublox requirements:
>>install ublox
>>
>>roslaunch ublox_gps ublox_device_beast.launch param_file_name:=alice param_file_dir:=/home/beast/catkin_ws/src/ublox/ublox_gps/config
>> add user to group for ttyACM0
>>> sudo usermod -a -G dailout beast (check what the group is first, dailout was the group in this example. beast is the name of the user)
>>>

>Ntrip client
>>install ntrip client and configure
