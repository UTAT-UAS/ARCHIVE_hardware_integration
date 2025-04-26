#!/usr/bin/env bash

bin_dir="$HOME/bin"
old_dir="$bin_dir/.aeac"

mkdir -p $bin_dir
mkdir -p $old_dir

clear_bashrc() {
    sed -i '/#==aeac/d' $HOME/.bashrc
}

append_bashrc() {
    cp $HOME/.bashrc $HOME/bin/.aeac/.old_bashrc
    echo "#==aeac mode tailnet" >>$HOME/.bashrc
    echo "# export ROS_MASTER_URI=http://10.5.0.2:11311 #==aeac-antenna" >>$HOME/.bashrc
    echo "# export ROS_IP=10.5.0.2 #==aeac-antenna" >>$HOME/.bashrc
}

create_bashrc() {
    clear_bashrc
    append_bashrc
}

system="drone"
script_name="$HOME/bin/aeac"
script=aeac

if [[ $USER == "uas-aeac" ]]; then # guess default
    system="gcs"
    script_name="$HOME/bin/gcs"
    script=gcs
fi

system_temp="input"
while true; do
    echo "choose system - [g]cs, [d]rone"
    read -p "(default $system): " system_temp
    if [[ -z $system_temp ]]; then
        break
    fi
    if [[ $system_temp =~ ^g$ ]]; then
        system="gcs"
        echo "  gcs"
        script_name="$HOME/bin/gcs"
        script=gcs
        break
    fi
    if [[ $system_temp =~ ^d$ ]]; then
        system="drone"
        echo "  drone"
        script_name="$HOME/bin/aeac"
        script=aeac
        break
    fi
    echo -e $tred"invalid input"$rst
done

if [[ -f $script_name ]]; then
    read -p "Do you want to update the script $script_name? (y/n): " response

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
