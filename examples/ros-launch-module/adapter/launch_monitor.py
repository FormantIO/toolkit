
import subprocess
import signal
import os
import psutil

def list_ros_launches():
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

print(list_ros_launches())
print(determine_launch_file(list_ros_launches()[0]))