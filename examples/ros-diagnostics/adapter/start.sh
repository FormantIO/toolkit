#! /bin/bash

### Fix ROS install on weird agent docker deployments
# apt-get install python3-pip
# apt-get purge python-rospkg && apt-get update && apt-get install -y ipython python-rospkg
# python3 -m pip install -r requirements.txt

### Source Availiable ROS
for dir in `ls /opt/ros | sort -r`; do
    if [ -f "/opt/ros/$dir/setup.bash" ]; then
        echo "Sourcing /opt/ros/$dir/setup.bash."
        . /opt/ros/$dir/setup.bash
        break
    fi
done
source config.env
python3 main.py