#!/usr/bin/env bash

tblk='\e[0;30m' # Black - Regular
tred='\e[0;31m' # Red
tgrn='\e[0;32m' # Green
tylw='\e[0;33m' # Yellow
tblu='\e[0;34m' # Blue
tpur='\e[0;35m' # Purple
tcyn='\e[0;36m' # Cyan
twht='\e[0;37m' # White
rst='\e[0m'     # Text Reset

print_help() {
    echo -e $tcyn"AEAC AUTO LAUNCHER"$rst
    echo "Running without args starts the interactive launcher"
    echo ""
    echo "ARGS:"
    echo "-a AUTO       Auto launch using a set string"
    exit 0
}

wait_for_tmux() {
    while true; do
        TMUXSESSIONS=$(tmux list-sessions 2>&1)
        if [[ ! $TMUXSESSIONS =~ ^no ]]; then
            sleep 0.1
            break
        fi
        sleep 0.1
    done
}

ir_launch() {
    wait_for_tmux
    tmux new-window
    tmux new-window
    tmux new-window
    tmux send-keys -t "0:0" "roslaunch simulation_testing real_ir.launch"
    tmux send-keys -t "0:0" "ENTER"
    sleep 1
    tmux send-keys -t "0:1" "rosrun computer_vision ir_detector.py"
    tmux send-keys -t "0:1" "ENTER"
    tmux send-keys -t "0:1" "rosrun simulation_testing test_task1.py"
}

bucket_launch() {
    wait_for_tmux
    tmux new-window
    tmux new-window
    tmux new-window
    tmux new-window
    tmux new-window
    tmux send-keys -t "0:0" "OAK"
    tmux send-keys -t "0:0" "ENTER"
    sleep 3
    tmux send-keys -t "0:1" "roslaunch simulation_testing real_cam.launch"
    tmux send-keys -t "0:1" "ENTER"
    sleep 1
    tmux send-keys -t "0:2" "rosrun rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:="
    tmux send-keys -t "0:2" "ENTER"
    tmux send-keys -t "0:3" "rosrun computer_vision bucket_detector.py"
    tmux send-keys -t "0:3" "ENTER"
    tmux send-keys -t "0:4" "rosrun simulation_testing test_"
}

auto_launch() {
    case ${1} in
    ir)
        ir_launch &
        tmux
        ;;
    bu)
        bucket_launch &
        tmux
        ;;
    esac
    exit 0
}

if [[ ! -z $TMUX ]]; then
    echo -e $tred"Running inside a tmux session forbidden"$rst
    exit 1
fi

TMUXSESSIONS=$(tmux list-sessions 2>&1)
if [[ ! $TMUXSESSIONS =~ ^no ]]; then
    echo -e $tred"Running script with existing tmux sessions not supported"$rst
    exit 1
fi

while getopts a:h opts; do
    case ${opts} in
    a) auto_launch $OPTARG ;;
    h) print_help ;;
    esac
done

echo -e $tgrn"AEAC AUTO LAUNCHER"$rst

# template
# VARNAME="input"
# while true; do
#     read -p "" VARNAME
#     if [[ $VARNAME =~ ^$ ]]; then
#         break
#     fi
#     echo -e $tred"invalid input"$rst
# done

flightstack="input"
while true; do
    read -p "Run flight stack [y] or just run mavros [any input]" flightstack
    if [[ ! $flightstack =~ ^y$ ]]; then
        (
            wait_for_tmux
            tmux new-window
            tmux send-keys -t "0:0" "roslaunch simulation_testing mavros.launch"
            tmux send-keys -t "0:0" "ENTER"
        ) &
        tmux
        exit 0
    fi
    echo -e $tred"invalid input"$rst
done

# TODO other input types

camera="input"
while true; do
    read -p "Luxonis [l], IR [i], None [n]: " camera
    if [[ $camera =~ ^[lin]{1}$ ]]; then
        break
    fi
    echo -e $tred"invalid input"$rst
done

echo $camera
