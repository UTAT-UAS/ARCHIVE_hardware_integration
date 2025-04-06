#!/usr/bin/env bash

mkdir -p ~/bin

script_name="$HOME/bin/aeac"

if [[ -f $script_name ]]; then
    read -p "Do you want to update the script? (y/n): " response

    if [[ "$response" == "y" || "$response" == "Y" ]]; then
        cp ./aeac $HOME/bin
        echo "Script updated."
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
    echo "Script installed."
    echo "Add $HOME/bin to your path e.g. \"export PATH=\$PATH:~/bin\" in your .bashrc"
fi
