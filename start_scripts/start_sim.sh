#!/bin/bash

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
ABS_SCRIPT_DIR=$(readlink -f "$SCRIPT_DIR")
NAME_WINDOW_1=""

setup_bash_path="$ABS_SCRIPT_DIR/../../devel/setup.bash"

session="self_racing_rc_platform-all"

tmux new-session -d -s $session

window=0
tmux rename-window -t $session:$window 'Window1'

# splitting the pane 0 vertically
tmux split-window -t 0 -v
# splitting the pane 0 vertically
tmux split-window -t 0 -v
tmux select-layout even-vertical
# splitting the pane 0 horizontally
tmux split-window -t 0 -h
# splitting the pane 2 horizontally
tmux split-window -t 2 -h
# splitting the pane 4 horizontally
tmux split-window -t 4 -h

# roscore
tmux select-pane -t 0
tmux send-keys 'roscore' C-m
# sleeping 10 seconds so that roscore has time to start
echo "Waiting 10 seconds for roscore to start..."
sleep 10

# gps listener
tmux select-pane -t 2
tmux send-keys "cd '$ABS_SCRIPT_DIR/../vehicle_sim/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun vehicle_sim sim.py" C-m

# map publisher
tmux select-pane -t 3
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomous_software_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg map_publisher.py" C-m

# waypoint publisher
tmux select-pane -t 4
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomous_software_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg waypoints_publisher.py _waypoints_file:=rex_manor_parking_lot_waypoints_speed.txt" C-m

# foxglove bridge
tmux select-pane -t 5
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomous_software_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "roslaunch foxglove_bridge foxglove_bridge.launch" C-m

window=1
tmux new-window -t $session:$window -n 'Window2'

# splitting the pane 0 vertically
tmux split-window -t 0 -v
# splitting the pane 0 vertically
tmux split-window -t 0 -v
tmux select-layout even-vertical
# splitting the pane 0 horizontally
tmux split-window -t 0 -h
# splitting the pane 2 horizontally
tmux split-window -t 2 -h
# splitting the pane 4 horizontally
tmux split-window -t 4 -h

# vehicle state publisher
tmux select-pane -t 0
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomous_software_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg vehicle_state_publisher.py" C-m

# target generator
tmux select-pane -t 1
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomous_software_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg target_generator.py" C-m

# lateral controller
tmux select-pane -t 2
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomous_software_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg lateral_controller.py" C-m

# longitudinal controller
tmux select-pane -t 3
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun longitudinal_controller longitudinal_controller.py" C-m

# attaching the session
tmux attach-session -t $session
