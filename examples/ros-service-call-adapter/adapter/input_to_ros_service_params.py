
import secrets
from tracemalloc import get_object_traceback

import utils
import json
import rostopic

# print(utils.get_ROS_format_keyed("random"))

def parse(input: str):
    """
    Parse the service call adapter input
    into a kwargs dictionary of parameters
    and their respective values
    """

    # JSOify the input string
    input = json.loads(input)
    
    # get the name of the service and its associated parameters
    service_name = list(input.keys())[0]
    service_params = input[service_name]

    # get the keyed format of the ros service request message
    service_param_definitions = utils.get_ROS_format_keyed(service_name)

    # Go through all the passed parameters, evaluating then, and 
    # return the kwargs to the caller
    param_objects = {}  
    for param in service_params:
        param_objects[param] = _get_param_obj(service_params[param], service_param_definitions[param], param) 

    return param_objects

def _get_param_obj(param_value, param_def, parent_name):
    """
    This function takes values and a type definition 
    and then converts it to the internal ros representation.
    """

    # Base case: the value of the current parameters 
    #       is primitive, and therefore can be evaluated easily.    
    if not isinstance(param_value, dict)\
        and not isinstance(param_def['type'], dict):
        arr_type = '[]' in param_def['ros_type']
        pytype = utils.ROS_type_to_python(param_def['ros_type'].replace('[]', ''))
        if arr_type:
            return [pytype(value) for value in param_value] 
        return pytype(param_value)
    
    # Recursive case: The parameter value is a nested message and thus
    #       we must evaluate the inner message parameters first
    new_param_values = {}
    for param in param_value:
        new_param_values[param[len(parent_name)+1:]] = _get_param_obj(param_value[param], param_def['type'][param], param)

    # Get the type of the parameter and then instantiate it
    # with the parameter values that we have already evaluated. 
    ros_type_obj = utils.get_message_class(param_def['ros_type'])  
    param_type_obj = ros_type_obj(**new_param_values)
    
    return param_type_obj
  



with open('sample.json', 'r') as f: 
    test = json.load(f)

test = json.dumps(test) 

print(parse(test)) 