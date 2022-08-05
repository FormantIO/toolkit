
import abc
from typing import List

class ROSLaunchException(Exception):

    def __init__(self):
        pass

class ROSLaunch(abc.ABC):

    def __init__(self):
        pass

    @abc.abstractmethod
    def get_args(self) -> List[str]:
        """
        Get the arguments for the service 
        """
        pass

    @abc.abstractmethod
    def is_running(self) -> bool:
        """
        Return true if the associated launch is currently running
        """
        pass

    @abc.abstractmethod
    def launch(self):
        """
        Launch the abstract method if this ROSLaunch is launch-able

        Raises ROSLaunchException on error
        """
        pass

    @abc.abstractmethod
    def kill(self):
        """
        Kill the launch that is represented by this ROS launch

        Raises ROSLaunchException on error
        """
        pass

    @abc.abstractmethod
    def is_external(self) -> bool:
        """
        is_external returns true if a the associated has no associated ros
        launch handle.
        """
        pass

    @abc.abstractmethod
    def is_internal(self) -> bool:
        """
        is_internal returns true if this launch was launched within this
        module and its launch handle was saved. 
        """
        pass

    @abc.abstractmethod
    def is_restartable(self) -> bool:
        """
        True if it is possible to restart the designated ROS launch. 

        It is possible to kill any ROS launch, but may not be 
        possible to start a killed launch again from this module. 
        """
        pass

