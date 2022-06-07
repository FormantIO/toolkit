
def ROS_to_json_schema_type_conversion(intype: str):
    """Convert a ROS type to it's corresponding python type."""
    mapping = {
        "bool":"bool",
        "int8":"integer",
        "uint8":"integer",
        "int16":"integer",
        "uint16":"integer",
        "int32":"integer",
        "uint32":"integer",
        "int64":"integer",
        "uint64":"integer",
        "float32":"number",
        "float64":"number",
        "string":"string",
        "time":"string",
        "duration":"string"
    }

    output = {}
    intype_stripped = intype.replace("[]","").lstrip().rstrip()
    output_type = mapping[intype_stripped] if not "[]" in intype else "array"
    
    output["type"] = output_type
    if output_type == "array":
        output["items"] = {"type" : mapping[intype_stripped]}

    return output

def ROS_type_to_JSON_schema(ros_type):

    if not isinstance(ros_type["type"], list):
        return ROS_to_json_schema_type_conversion(ros_type["type"])

    # If we get here, it means that the type has sub types / child types
    output = {}
    output["type"] = "object"
    output["properties"] = {}
    for param in ros_type["type"]:
        output["properties"][param["name"]] = ROS_type_to_JSON_schema(param)
    
    return output



def ROS_format_to_JSON_schema(service_name, ros_format):
    
    schema = {}

    schema["title"] = service_name
    schema["type"] = "object"
    properties = {}

    for param in ros_format:
        properties[param["name"]] = ROS_type_to_JSON_schema(param) 

    schema["properties"] = properties
    return schema