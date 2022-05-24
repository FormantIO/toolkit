
import rosservice
import rospy
import rosgraph
import rostopic
import roslib
from logger import getLogger

logger = getLogger()


def current_services():
    """Return the currently active list of services."""
    return set(rosservice.get_service_list())


def service_call(service_name, *args):
    """Call a ROS service and return the result."""
    if service_name not in current_services():
        logger.warn(
            f"Service {service_name} not online. Dropping service call.")
        return

    rospy.wait_for_service(service_name)
    result = None
    try:
        service = rospy.ServiceProxy(
            service_name, get_service_type_obj(service_name))
        try:
            result = service(*args)  # Call the service.
        except TypeError as e:
            logger.info(f"Failed to execute service call. {e}")
            result = None
    except rospy.ServiceException as e:
        logger.info("Service Call Failed.")

    return result


def is_valid_ros_service(service_name):
    """Returns a boolean which indicates if a service is valid."""
    if not rosgraph.is_master_online():
        return False

    try:
        datatype = rosservice.get_service_class_by_name(service_name)
        rostopic.get_topic_type()

    except rosservice.ROSServiceException:
        logger.info(
            f"Cannot validate service: {service_name}, ROS master could not be reached.")
        return False

    if not datatype:
        return False

    return True


def get_service_type_obj(service_name: str):
    return rosservice.get_service_class_by_name(service_name)
