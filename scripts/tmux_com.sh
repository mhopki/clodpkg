#!/bin/bash

# Name of the tmux session
SESSION_NAME="beast_com_session"

# Create a new tmux session
tmux new-session -d -s $SESSION_NAME

# Create a new window and run the first ROS launch command
tmux rename-window -t $SESSION_NAME:0 'roscore'  # First window: roscore
tmux send-keys -t $SESSION_NAME:0 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; roscore'

# Create additional windows for each ROS launch/node

# Window 1: First ROS launch
tmux new-window -t $SESSION_NAME:1 -n 'joystick'
tmux send-keys -t $SESSION_NAME:1 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; roscd clod_pkg/scripts/; ./com_launch.sh'

# Window 2: Second ROS launch
tmux new-window -t $SESSION_NAME:2 -n 'waypoints'
tmux send-keys -t $SESSION_NAME:2 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; rosrun clod_pkg waypoint_sender.py'

# Window 3: Third ROS node
tmux new-window -t $SESSION_NAME:3 -n 'path_code'
tmux send-keys -t $SESSION_NAME:3 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; rosrun clod_pkg rrt.py'

# Window 4: Third ROS node
tmux new-window -t $SESSION_NAME:4 -n 'KILL'
tmux send-keys -t $SESSION_NAME:4 'source ~/catkin_ws/src/clodpkg/scripts/rosip.sh; tmux kill-session -t beast_com_session'

# Attach to the session
tmux attach-session -t $SESSION_NAME
