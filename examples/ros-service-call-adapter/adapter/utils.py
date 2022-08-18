
import logging

import rosgraph
import roslib
import rosmsg
import rospy
import rosservice

from indented_parser import parse_indented_string
from indented_to_ros import parse_indented_as_ros

logger = logging.getLogger()


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
        except Exception as e:
            logger.info(f"Failed to execute service call. {e}")
            result = None
    except rospy.ServiceException as e:
        logger.info(f"Service Call Failed. Reason: {e}")

    return result


def is_valid_ros_service(service_name):
    """Returns a boolean which indicates if a service is valid."""
    if not rosgraph.is_master_online():
        return False

    try:
        datatype = rosservice.get_service_class_by_name(service_name)
        # rostopic.get_topic_type()

    except rosservice.ROSServiceException:
        logger.info(
            f"Cannot validate service: {service_name}, ROS master could not be reached.")
        return False

    if not datatype:
        return False

    return True


def get_service_type_obj(service_name: str):
    return rosservice.get_service_class_by_name(service_name)


def get_ROS_service_type_str(service_name: str):
    try:
        return rosservice.get_service_type(service_name)
    except rosservice.ROSServiceException:
        return None


def get_ROS_format(service: str):
    """"""

    try:
        type_str = get_ROS_service_type_str(service)
        srv_text = rosmsg.get_srv_text(type_str)
    except rosmsg.ROSMsgException:
        logger.warn(f"Failure getting srv text for {service}")
        return None

    parsed_from_indented_text = parse_indented_string(srv_text)
    ros_formatted = parse_indented_as_ros(parsed_from_indented_text)

    return ros_formatted


def get_ROS_format_keyed(service: str):
    service = rospy.resolve_name(service)
    ros_formatted_list = get_ROS_format(service)

    output = {}

    for param in ros_formatted_list:
        output[param['name']] = _gen_key_output(param)

    return output


def _gen_key_output(type_dict):

    if not isinstance(type_dict['type'], list):
        return type_dict

    if type_dict['type']:
        new_output_type = {}

        for param in type_dict['type']:
            new_output_type[param['name']] = _gen_key_output(param)

        type_dict['type'] = new_output_type

        return type_dict

    return {}


def get_message_class(message_class_str: str):
    return roslib.message.get_message_class(message_class_str)


def ROS_type_to_python(ros_type: str):
    return {
        "bool": bool,
        "int8": int,
        "uint8": int,
        "int16": int,
        "uint16": int,
        "int32": int,
        "uint32": int,
        "int64": int,
        "uint64": int,
        "float32": float,
        "float64": float,
        "string": str,
        "time": rospy.Time,
        "duration": rospy.Duration
    }[ros_type]
