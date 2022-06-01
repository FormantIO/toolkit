#!/usr/bin/python3

from os.path import exists

file_exists = exists("/var/lib/formant/.bashrc")

if not file_exists:
    exit(0)

catkin_ws = ""

with open("/var/lib/formant/.bashrc", "r") as f:
    for line in f:
        if "export CATKIN_WS" in line:
            catkin_ws = line.split("=")
            if len(catkin_ws) != 2:
                exit(0)

            catkin_ws = catkin_ws[1].lstrip().rstrip().replace("\n", "")

if exists(f"{catkin_ws}/devel/setup.bash"):
    print(f"{catkin_ws}/devel/setup.bash")
    # print("")
