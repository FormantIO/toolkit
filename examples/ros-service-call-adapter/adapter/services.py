
import json
import time

from yaml import parse

import rosmsg
import rosservice
from formant.sdk.agent.v1.client import Client as FormantClient

from logger import getLogger
from indented_parser import parse_indented_string
from indented_to_ros import convert_to_ros

logger = getLogger

class ServiceChecker:
    """Provides an interface to use the ROS service checker which posts data to ros.services.json"""

    def __init__(self):
        """Initialize the ServiceChecker"""
        services = ServiceChecker._get_running_services()
        
        self._fclient = FormantClient()
        self._shutdown_signal = self._shutdown
        #self._service_thread = threading.Thread(target=self._run, daemon=True)
        #self._service_thread.start()
        self._started = False
        self._is_shutdown = True 
        self._data_to_post = False

    def start(self):
        """Starts the service checker thread."""
        self._is_shutdown = False
        #self._service_thread.start()
        self._started = True

        self._fclient.register_command_request_callback(self._check_services, ["ros.services.update-services"])
        self._check_services()

    def shutdown(self):
        """Kills the ServiceChecker."""
        self._is_shutdown = True
        # self._service_thread.join()

    def _shutdown(self):
        """return True if self._is_shutdown is set."""
        return self._is_shutdown  

    def _run(self):
        """Run the ServiceChecker in a loop which checks for services then reports the JSON schema"""
        while not self._shutdown_signal():
            self._check_services()
            self._post_json()
            time.sleep(2) 

    def _check_services(self, *_):
        """Check current published services then report the JSON schema for the services."""
        logger.debug("Checking Running services")
        service_names = self._get_running_services()
        
        services = {}
        

        for service_name in service_names:
            service = RosService(service_name) 
            if not service.is_valid():
                continue
            services[service_name] = service.request_args()

        self._services_json = json.dumps(services)
        self._data_to_post = True
    
    def _post_json(self):
        if not self._data_to_post:
            return
        
        logger.debug(f"Sending {self._services_json}")

        try:
            self._fclient.post_json("ros.services.json", self._services_json)
        except Exception:
            logger.warn("Error posting data to ros.services.json")
        self._data_to_post = False

    @staticmethod
    def _get_running_services():
        """Get current published services."""
        try:
            return rosservice.get_service_list()
        except rosservice.ROSServiceException:
            return []

    def _get_service_type(self, service_name):
        return rosservice.get_service_type(service_name)


class RosService:
    """The RosService acts as a wrapper for services and provides an interface for accessing service data."""

    def __init__(self, service_name):
        """Initialize the RosService class and parse the service."""
     
        self._is_valid = True
        self._service_name = service_name

        try:
            self._service_type_str = rosservice.get_service_type(service_name) 

            # Check if the service is running. If not, abort
            if service_name not in set(rosservice.get_service_list()):
                self._is_valid = False
                return
        except Exception:
            self._is_valid = False

    def is_valid(self):
        """Return True if all the parsing was successful"""
        return self._is_valid

    def request_as_json(self):          
        """Return the service arguments as a JSON string"""

        return json.dumps(self.request_args())

    def request_args(self):
        """Request the args for the service and the associated types."""
        srv_text = rosmsg.get_srv_text(self._service_type_str)
        return convert_to_ros(parse_indented_string(srv_text))

class RosServiceOld:
    """The RosService acts as a wrapper for services and provides an interface for accessing service data."""

    def __init__(self, service_name):
        """Initialize the RosService class and parse the service."""
     
        self._is_valid = True
        self._service_name = service_name

        try:
            self._service_type_str = rosservice.get_service_type(service_name) 

            # Check if the service is running. If not, abort
            if service_name not in set(rosservice.get_service_list()):
                self._is_valid = False
                return
            
            self._args = self._get_service_args_and_types()
        except Exception:
            self._is_valid = False

    def is_valid(self):
        """Return True if all the parsing was successful"""
        return self._is_valid

    def request_as_json(self):          
        """Return the service arguments as a JSON string"""

        return json.dumps(self.request_args())

    def request_args(self):
        """Request the args for the service and the associated types."""
        srv_text = rosmsg.get_srv_text(self._service_type_str)
        return convert_to_ros(parse_indented_string(srv_text))

def is_primitive(type_str):
    pass
