#!/usr/bin/env bash

if [ -f ~/.savedrc ]; then
    echo "ERROR: Environment currently active"
    exit 1
fi

cp ~/.bashrc ~/.savedrc
cat .bashrc >>~/.bashrc

echo "Environment setup complete, re-source .bashrc"
echo "RESET BEFORE PULLING CHANGES TO SETUP/RESET SCRIPT"
