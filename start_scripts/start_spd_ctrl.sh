#!/bin/bash

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
ABS_SCRIPT_DIR=$(readlink -f "$SCRIPT_DIR")

setup_bash_path="$ABS_SCRIPT_DIR/../../devel/setup.bash"
session="self_racing_rc_platform-gui"

cd ~/workspace/self_racing_rc_platform_ws/
tmux new-session -d -s $session

# splitting the pane 0 vertically
tmux split-window -t 0 -v
# splitting the pane 0 vertically
tmux split-window -t 0 -v
tmux split-window -t 0 -v

tmux select-layout even-vertical


# splitting the pane 0 horizontally
tmux split-window -t 0 -h
# splitting the pane 2 horizontally
tmux split-window -t 2 -h
# splitting the pane 4 horizontally
tmux split-window -t 4 -h
# splitting the pane 6 horizontally


# roscore
tmux select-pane -t 0
# tmux send-keys 'source /home/antoine/.bashrc' C-m
tmux send-keys 'roscore' C-m
# sleeping 10 seconds so that roscore has time to start
echo "Waiting 5 seconds for roscore to start..."
sleep 4

# Rosbag play /gps_info
tmux select-pane -t 1
tmux send-keys "cd '$ABS_SCRIPT_DIR/../bags/'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosbag play parking_lot_1.bag --topics /gps_info" C-m

# Printout /gps_info 
tmux select-pane -t 2
tmux send-keys "cd '$ABS_SCRIPT_DIR/../../'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rostopic echo /gps_info" C-m

# Running Vehicle State Publisher
tmux select-pane -t 3
tmux send-keys "cd '$ABS_SCRIPT_DIR/../../'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg vehicle_state_publisher.py" C-m

# Running Controller
tmux select-pane -t 4
tmux send-keys "cd '$ABS_SCRIPT_DIR/../../'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg controller.py" C-m

#Printout /vehicle_command topic
tmux select-pane -t 5
tmux send-keys "cd '$ABS_SCRIPT_DIR/../../'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rostopic echo vehicle_command" C-m

# Run slider GUI
tmux select-pane -t 6
tmux send-keys "cd '$ABS_SCRIPT_DIR/../../'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg slider_publisher.py" C-m
tmux attach-session -t $session