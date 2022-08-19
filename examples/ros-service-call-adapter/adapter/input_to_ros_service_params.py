
import json
import logging

import rospy

import utils

logger = logging.getLogger()

def parse(input: str):
    """
    Parse the service call adapter input
    into an args dictionary of parameters
    and their respective values
    """

    # JSOify the input string
    input = json.loads(input)

    # get the name of the service and its associated parameters
    service_name = rospy.resolve_name(list(input.keys())[0])
    service_params = input[list(input.keys())[0]]

    if not utils.is_valid_ros_service(service_name):
        logger.warn("Invalid service")
        raise Exception("Invalid service name")

    # get the keyed format of the ros service request message
    service_param_definitions = utils.get_ROS_format_keyed(service_name)
    ordered_params = utils.get_ROS_format(service_name)

    # Go through all the passed parameters, evaluating then, and
    # return the kwargs to the caller
    param_objects = {}
    for param in service_params:
        param_objects[param] = _get_param_obj(
            service_params[param], service_param_definitions[param], param)

    # The final step is to create an ordering for the parameters
    # so that the service can be called
    param_objects_ordered = []

    for param in ordered_params:
        name = param['name']
        param_objects_ordered.append(param_objects[name])

    return (service_name, param_objects_ordered)


def _get_param_obj(param_value, param_def, parent_name):
    """
    This function takes values and a type definition 
    and then converts it to the internal ros representation.
    """

    # Base case: the value of the current parameters
    #       is primitive, and therefore can be evaluated easily.
    if not isinstance(param_value, dict)\
            and not isinstance(param_def['type'], dict):
        arr_type = param_def['ros_type'][-1] == ']'
        pytype = utils.ROS_type_to_python(
            get_primitive_type(param_def['ros_type']))
        if arr_type:
            return [pytype(value) for value in param_value]
        return pytype(param_value)

    # Recursive case: The parameter value is a nested message and thus
    #       we must evaluate the inner message parameters first
    new_param_values = {}
    for param in param_value:
        new_param_values[param[len(parent_name)+1:]] = _get_param_obj(
            param_value[param], param_def['type'][param], param)

    # Get the type of the parameter and then instantiate it
    # with the parameter values that we have already evaluated.
    ros_type_obj = utils.get_message_class(param_def['ros_type'])
    param_type_obj = ros_type_obj(**new_param_values)

    return param_type_obj

def get_primitive_type(type_def):
    """Given a type definition of an array type, get the array item type."""
    if type_def[-1] != ']':
        return type_def
    return type_def[:type_def.find("[")]
