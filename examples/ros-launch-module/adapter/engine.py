
from typing import Dict, List

from launch import ROSLaunch, ROSLaunchException 
from launch_monitor import list_ros_launches, determine_launch_file

class RosLaunchEngine: 
    
    def __init__(self):
        pass

    def currently_running(self) -> List[str]:
        """
        Return all the currently running ros launches
        """
        launches = [determine_launch_file(p) for p in list_ros_launches()]
        return [l for l in launches if l is not None] 

    def available_launch_files(self) -> List[str]: 
        """
        Return a list of all the found launch files that the 
        user may launch. 
        """
        pass

    def launch(self, launch_file: str, externally=False):
        """
        Launch the specified launch file. 

        @param launch_file: str - The name of the ros launch that we would
                                    like to launch
        @param externally: bool - Should we launch the file as a standalone
                                    process?
                                    Note: If the program dies, the launch will
                                            still live if its launched externally.
        """
        pass

    def kill(self, identifier):
        """
        Kill the specified ros launch associated with identifier.
        This can be either a PID or a string. 

        @note: Use PID when possible. If a roslaunch file was launched,
               multiple times, then it is not possible to kill the launch file
               by name. 
        """
        pass

    def restart(self, identifier):
        """
        Restart the specified launch associate with the identifier.

        @note: Use PID when possible. If a roslaunch file was launched,
               multiple times, then it is not possible to restart by the name alone.
        """
        pass
    
    def status(self, identifier) -> Dict:
        """
        Return a dictionary which holds the status information
        of the ros launch with the associated identifier

        @note: Use PID when possible. If a roslaunch file was launched,
               multiple times, then it is not possible to get the status 
               of a launch via name alone.
        """
        pass