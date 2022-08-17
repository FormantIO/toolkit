#! /bin/bash

### Source Availiable ROS
if [ -d "/opt/ross" ]; then
    export PYTHONUNBUFFERED=true
    pip3 install setuptools
    pip3 install wheel
    python3 -m pip install -r requirements.txt
    for dir in $(ls /opt/ros | sort -r); do
        if [ -f "/opt/ros/$dir/setup.bash" ]; then
            echo "Sourcing /opt/ros/$dir/setup.bash."
            . /opt/ros/$dir/setup.bash
            break
        fi
    done
    source config.env
    python3 main.py
fi
