#!/bin/sh

if [ $# -eq 0 ]; then
  echo "Input mav number(integer) as first argument"
  exit 1
fi

MAV_ID=$1
if echo $MAV_ID | grep -Eq '^[+-]?[0-9]+$'
then
  echo "Recording bag file for MAV $MAV_ID"
else
  echo "Input mav number(integer) as first argument"
  exit 1
fi

MAV_NAME="BEAST"
echo "Robot Name $MAV_NAME"

bag_folder="~/"

if [ ! -d "$bag_folder" ]; then
  echo "*** WARNING *** target folder not found, recording locally"
  cd ~/
else
  echo 'Bag files stored at' $bag_folder
  cd $bag_folder

  #Print out %Used SD card
  USED_PERCENT=$(df --output=pcent $bag_folder | awk '/[0-9]%/{print $(NF)}' | awk '{print substr($1, 1, length($1)-1)}')
  echo 'Storage' ${USED_PERCENT} '% Full'
fi


GPS_TOPICS="
/$MAV_NAME//mavros/global_position/global
/$MAV_NAME/quadrotor_ukf/control_odom_throttled
/$MAV_NAME/quadrotor_ukf/imu_bias"

RGBD_TOPICS="
/$MAV_NAME/camera/image_raw"

VIO_TOPICS="
/$MAV_NAME/camera/odom/sample
/$MAV_NAME/camera/fisheye1/image_raw
/$MAV_NAME/camera/fisheye2/image_raw"


ALL_TOPICS=$GPS_TOPICS$RGBD_TOPICS$VIO_TOPICS

BAG_STAMP=$(date +%F-%H-%M-%S-%Z)
CURR_TIMEZONE=$(date +%Z)

BAG_NAME=$BAG_STAMP-BEAST.bag
BAG_PREFIX=BEAST-${CURR_TIMEZONE}

eval rosbag record -b512 $ALL_TOPICS -o $BAG_PREFIX