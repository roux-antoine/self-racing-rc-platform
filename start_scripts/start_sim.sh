#!/bin/bash

SCRIPT_DIR="$(dirname "${BASH_SOURCE[0]}")"
ABS_SCRIPT_DIR=$(readlink -f "$SCRIPT_DIR")
NAME_WINDOW_1=""

setup_bash_path="$ABS_SCRIPT_DIR/../../devel/setup.bash"

session="self_racing_rc_platform-sim"

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

# Simulator
tmux select-pane -t 0
tmux send-keys "cd '$ABS_SCRIPT_DIR/../utils/vehicle_sim_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun vehicle_sim_pkg sim.py" C-m

# map publisher
tmux select-pane -t 1
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomy_software/localization_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun localization_pkg map_publisher.py _map_file_name:=MapSanMateoP1.kml" C-m

# waypoint publisher
tmux select-pane -t 2
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomy_software/localization_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun localization_pkg waypoints_publisher.py _waypoints_file:=waypoints_P1_straight_1_detail_3m_speed.txt" C-m

# vehicle state publisher
tmux select-pane -t 3
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomy_software/localization_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun localization_pkg vehicle_state_publisher.py" C-m

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

# target generator
tmux select-pane -t 0
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomy_software/planning_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun planning_pkg target_generator.py" C-m

# lateral controller
tmux select-pane -t 1
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomy_software/controllers_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun controllers_pkg lateral_controller.py" C-m

# Longitudinal controller
tmux select-pane -t 2
tmux send-keys "cd '$ABS_SCRIPT_DIR/../autonomy_software/controllers_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun controllers_pkg longitudinal_controller.py" C-m

# attaching the session
tmux attach-session -t $session
