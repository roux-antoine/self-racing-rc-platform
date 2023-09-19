#!/bin/bash

setup_bash_path='/home/nico/workspace/self_racing_rc_platform_ws/devel/setup.bash'
session="self_racing_rc_platform-gui"

cd ~/workspace/self_racing_rc_platform_ws/
tmux new-session -d -s $session

# splitting the pane 0 vertically
tmux split-window -t 0 -v
# splitting the pane 0 vertically

tmux select-layout even-vertical


# splitting the pane 0 horizontally
tmux split-window -t 0 -h
# splitting the pane 2 horizontally
tmux split-window -t 2 -h
# splitting the pane 4 horizontally


# roscore
tmux select-pane -t 0
# tmux send-keys 'source /home/antoine/.bashrc' C-m
tmux send-keys 'roscore' C-m
# sleeping 10 seconds so that roscore has time to start
echo "Waiting 5 seconds for roscore to start..."
sleep 2

tmux select-pane -t 1
tmux send-keys "cd '/home/nico/workspace/self_racing_rc_platform_ws/'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg slider_publisher.py" C-m


tmux select-pane -t 2
tmux send-keys "cd '/home/nico/workspace/self_racing_rc_platform_ws/'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rostopic echo /teleop_speed" C-m




tmux attach-session -t $session