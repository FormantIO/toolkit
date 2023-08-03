def ROS_to_json_schema_type_conversion(intype: str):
    """Convert a ROS type to it's corresponding python type."""
    mapping = {
        "bool": "boolean",
        "int8": "integer",
        "uint8": "integer",
        "int16": "integer",
        "uint16": "integer",
        "int32": "integer",
        "uint32": "integer",
        "int64": "integer",
        "uint64": "integer",
        "float32": "number",
        "float64": "number",
        "string": "string",
        "byte": "integer"
        ## NOTE for ROS2 this will cause an issue as ROS2 byte doesn't accept an int.
    }

    output = {}
    intype = intype.lstrip().rstrip()

    array_type = intype[-1] == "]"

    if array_type:
        intype_stripped = intype[:intype.find("[")]
    else:
        intype_stripped = intype

    output_type = mapping[intype_stripped] if not array_type else "array"

    output["type"] = output_type
    if output_type == "array":
        output["items"] = {"type": mapping[intype_stripped]}
        if intype[-2] != '[':
            output["minItems"] = int(intype[-2])
            output["maxItems"] = int(intype[-2])

    return output


def ROS_type_to_JSON_schema(ros_type):
    """Convert a singular ros type to a JSON schema"""

    # and ros_type['type'] not in special_types:
    if not isinstance(ros_type["type"], list):
        output = ROS_to_json_schema_type_conversion(ros_type["type"])
        output["title"] = ros_type["name"]

        return output

    # If we get here, it means that the type has sub types / child types
    output = {}
    output["type"] = "object"
    output["properties"] = {}
    output["title"] = ros_type["name"]
    for param in ros_type["type"]:
        output["properties"][param["name"]] = ROS_type_to_JSON_schema(param)

    return output


def ROS_format_to_JSON_schema(service_name, ros_format):
    """Convert a JSON generated for ROS to a JSON schema document."""

    schema = {}

    schema["title"] = service_name
    schema["type"] = "object"
    properties = {}

    for param in ros_format:
        properties[param["name"]] = ROS_type_to_JSON_schema(param)

    schema["properties"] = properties
    return schema
