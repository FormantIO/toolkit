#! /bin/bash

if [ ! -d "/opt/ros" ]; then
    sleep 1d
fi

export PYTHONUNBUFFERED=true
pip3 install setuptools
pip3 install wheel
pip3 install rospkg
python3 -m pip install -r requirements.txt

### Source Availiable ROS
for dir in `ls /opt/ros | sort -r`; do
    if [ -f "/opt/ros/$dir/setup.bash" ]; then
        echo "Sourcing /opt/ros/$dir/setup.bash."
        . /opt/ros/$dir/setup.bash
        break
    fi
done
source config.env
pip3 install python-lzf
python3 main.py