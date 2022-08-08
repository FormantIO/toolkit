
import subprocess
import signal
import os
from typing import List
import psutil

def list_ros_launches() -> List[psutil.Process]:
    """
    Return a list of all currently running external ros launches.
    """
    ros_launches = [] 
    for proc in psutil.process_iter():
        try:
            # Get process name & pid from process object.
            processName = proc.name()
            processID = proc.pid

            if processName == "roslaunch":       
                ros_launches.append( proc )
                
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass

    return ros_launches

def determine_launch_file(proc: psutil.Process):
    """
    Given a proc object, determine the launch file name that 
    spawned the associated process.
    """

    cmd = proc.cmdline()

    for elt in cmd:
        if ".launch" in elt:
            return elt

def get_launch_dict(key="PID"):
    """
    Return a dictionary of launches. This maps either 
    the PID or launchfile-name to a list of launches. In the case of PID, 
    the list of launches will always be length 1. 
    """
    if key == "PID":
        def get_key(proc: psutil.Process):
            return proc.pid()
    else:
        def get_key(proc: psutil.Process):
            return determine_launch_file(proc) 

    launches = list_ros_launches()

    launch_dict = {}

    for launch in launches:
        launch_key = get_key(launch) 

        if launch_key not in launch_dict:
            launch_dict[launch_key] = []
        
        launch_dict[launch_key].append(launch) 

    return launch_dict

def is_running(identifier):
    pass

def get_proc(identifier):
    pass

