#!/bin/bash

# Session Name
SESSION="hsr-session"

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# Start new session in detached mode
tmux new-session -d -s $SESSION

tmux split-window -h -t "$SESSION"
tmux split-window -v -t "$SESSION:0.0"
tmux split-window -v -t "$SESSION:0.2"
tmux send-keys -t "$SESSION:0.0" 'roscore' C-m
sleep 2
# tmux send-keys -t "$SESSION:0.1" '' C-m
tmux send-keys -t "$SESSION:0.2" 'roslaunch suturo_bringup suturo_tab.launch use_rviz:=false' C-m
tmux send-keys -t "$SESSION:0.3" 'roslaunch giskardpy_ros giskardpy_hsr_standalone.launch debug_mode:=true' C-m

tmux new-window -t $SESSION
tmux split-window -h -t "$SESSION:1"
tmux split-window -v -t "$SESSION:1.0"
tmux split-window -v -t "$SESSION:1.2"
tmux send-keys -t "$SESSION:1.0" 'rosrun giskardpy_ros force_torque_raw_filter.py' C-m
tmux send-keys -t "$SESSION:1.1" 'rosrun giskardpy_ros gripper_sim.py' C-m
tmux send-keys -t "$SESSION:1.2" 'cd ~/bags/forcetorque; rosbag play -l forcetorque.bag' C-m # edit path to reflect local setup
tmux send-keys -t "$SESSION:1.3" 'cd ~/bags/handcamera; rosbag play -l handcamera.bag' C-m # edit path to reflect local setup

# Attach to the session
tmux attach-session -t "$SESSION:0"