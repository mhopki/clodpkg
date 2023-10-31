# clodpkg

Control for clodbuster car


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
