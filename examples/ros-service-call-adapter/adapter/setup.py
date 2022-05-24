#!/usr/bin/python

from os.path import exists

file_exists = exists("/var/lib/formant/.bashrc")

if not file_exists:
    exit(0)

catkin_ws = "" 

with open("/var/lib/formant/.bashrc", "r") as f:
    for line in f:
        if "export CATKIN_WS" in line:
            catkin_ws = line.split("=")[1].lstrip().rstrip().replace("\n","")

if exists(f"{catkin_ws}/devel/setup.bash"):
    print(f"{catkin_ws}/devel/setup.bash")
    # print("")
