#!/bin/bash

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
ABS_SCRIPT_DIR=$(readlink -f "$SCRIPT_DIR")
NAME_WINDOW_1=""

setup_bash_path="$ABS_SCRIPT_DIR/../../devel/setup.bash"

session="self_racing_rc_platform-infra"

tmux new-session -d -s $session

window=0
tmux rename-window -t $session:$window 'Window1'

# splitting the pane 0 vertically
tmux split-window -t 0 -v
# splitting the pane 0 vertically
tmux split-window -t 0 -v
# splitting the pane 0 vertically
tmux split-window -t 0 -v

# roscore
tmux select-pane -t 0
tmux send-keys 'roscore' C-m
# sleeping 10 seconds so that roscore has time to start
echo "Waiting 10 seconds for roscore to start..."
sleep 10

# foxglove bridge
tmux select-pane -t 1
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomy_software'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "roslaunch foxglove_bridge foxglove_bridge.launch" C-m

# rqt_reconfigure
tmux select-pane -t 2
tmux send-keys "rosrun rqt_reconfigure rqt_reconfigure" C-m
echo "Waiting 10 seconds for rqt_reconfigure to start..."
sleep 10

echo "Infra started. Exiting."
