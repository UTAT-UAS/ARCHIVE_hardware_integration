#!/usr/bin/env bash

if [ ! -f ~/.savedrc ]; then
    echo "ERROR: No environment active"
    exit 1
fi

cp ~/.savedrc ~/.bashrc
rm ~/.savedrc

echo "Environment reverted"i, re-source .bashrc"
