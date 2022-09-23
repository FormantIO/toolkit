#!/bin/bash

if [ ! -d "/opt/ros" ]; then
    sleep 1d
fi

export PYTHONUNBUFFERED=true
pip3 install setuptools
pip3 install wheel
pip3 install -r requirements.txt
/usr/bin/python3 main.py
