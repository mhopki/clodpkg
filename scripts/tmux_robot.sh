#!/bin/bash

# Name of the tmux session
SESSION_NAME="beast_robot_session"

# Create a new tmux session
tmux new-session -d -s $SESSION_NAME

# Create a new window and run the first ROS launch command
tmux rename-window -t $SESSION_NAME:0 'roscore'  # First window: roscore
tmux send-keys -t $SESSION_NAME:0 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; roscore' C-m

# Create additional windows for each ROS launch/node

# Window 1: First ROS launch
tmux new-window -t $SESSION_NAME:1 -n 'robot_launch'
tmux send-keys -t $SESSION_NAME:1 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; roscd clod_pkg/scripts/; sleep 5; ./robot_launch.sh' C-m

# Window 2: Second ROS launch
tmux new-window -t $SESSION_NAME:2 -n 'cam_segment'
tmux send-keys -t $SESSION_NAME:2 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; sleep 5; rosrun clod_pkg cam_segment_yolo.py' C-m

# Window 3: Second ROS launch
tmux new-window -t $SESSION_NAME:3 -n 'hyperspectral_photo'
tmux send-keys -t $SESSION_NAME:3 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; sleep 5; python3 ~/catkin_ws/src/clodpkg/src/hyperspectral_node.py' C-m

# Window 4: Third ROS node
tmux new-window -t $SESSION_NAME:4 -n 'yolo'
tmux send-keys -t $SESSION_NAME:4 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; sleep 5; roslaunch yolov7_ros yolov7-ag.launch' C-m

# Window 4: Third ROS node
tmux new-window -t $SESSION_NAME:5 -n 'gps-imu'
tmux send-keys -t $SESSION_NAME:5 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; sleep 5; rosrun sensor_fusion transform_imu.py' C-m

# Window 4: Third ROS node
tmux new-window -t $SESSION_NAME:6 -n 'gps-ekf'
tmux send-keys -t $SESSION_NAME:6 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; sleep 7; roslaunch sensor_fusion ekf.launch' C-m

# Window 5: Third ROS node
tmux new-window -t $SESSION_NAME:7 -n 'KILL'
tmux send-keys -t $SESSION_NAME:7 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; tmux kill-session -t beast_robot_session'

# Attach to the session
tmux attach-session -t $SESSION_NAME
