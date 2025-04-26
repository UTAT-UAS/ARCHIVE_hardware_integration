#!/usr/bin/env bash

bin_dir="$HOME/bin"
script_name="$HOME/bin/aeac"
old_dir="$bin_dir/.aeac"

mkdir -p $bin_dir
mkdir -p $old_dir

clear_bashrc() {
    # clear
}

append_bashrc() {
    cp $HOME/.bashrc $HOME/bin/.aeac/.old_bashrc
    cp $HOME/bin/aeac/.old_bashrc $HOME/bin/aeac/.wifi_bashrc
    echo "export ROS_MASTER_URI=http://10.5.0.2:11311" >>$HOME/bin/aeac/.wifi_bashrc
    echo "export ROS_IP=10.5.0.2" >>$HOME/bin/aeac/.wifi_bashrc
}

create_bashrc() {
    clear_bashrc
    append_bashrc
}

if [[ -f $script_name ]]; then
    read -p "Do you want to update the script? Ensure .bashrc restored first. (y/n): " response

    if [[ "$response" == "y" || "$response" == "Y" ]]; then
        cp ./aeac $HOME/bin
        create_bashrc
        echo "Script and .bashrc updated."
        echo "Ensure $HOME/bin is in your path e.g. \"export PATH=\$PATH:~/bin\""
    elif [[ "$response" == "n" || "$response" == "N" ]]; then
        echo "Script was not updated."
        exit 0
    else
        echo "Invalid response. Please enter 'y' or 'n'."
        exit 1
    fi
else
    cp ./aeac $HOME/bin
    create_bashrc
    echo "Script installed and .bashrc updated."
    echo "Add $HOME/bin to your path e.g. \"export PATH=\$PATH:~/bin\" in your .bashrc"
fi
