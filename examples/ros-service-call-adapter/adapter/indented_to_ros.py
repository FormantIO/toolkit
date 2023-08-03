
def flatten_ros_data_structure(ros_format):
    elements = []

    for param in ros_format:
        if isinstance(param["type"], list):
            elements += flatten_ros_data_structure(param["type"])
        else:
            elements.append(param)

    return elements


def parse_indented_as_ros(indented_json_format):
    converter = IndentedJsonToRosJson(indented_json_format)
    return converter.get_converted()


class IndentedJsonToRosJson:
    """
    Indented JSON to ROS Json provides a tool that will parse
    indented Json data into its ROS parts. The indented format is
    generated from parse_indented_string 
    """

    def __init__(self, indented_format):
        self._indented_format = indented_format

        self._ros_format = []
        self._convert()

    def get_converted(self):
        return self._ros_format

    def _convert(self):
        for param in self._indented_format:
            # Signifies the end of the request part of a ROS message
            if "---" in param["name"]:
                break
            if "=" in param["name"]:
                continue
            self._ros_format.append(self._convert_type(param))

    def _convert_type(self, type_obj, parent_name=""):
        """
        Convert a single parent child structure into a ROS formatted structure. 
        """

        ros_type, param_name = type_obj["name"].lstrip(
        ).rstrip().replace("\n", "").split(" ")
        p_name = f"{parent_name}.{param_name}" if len(
            parent_name) else param_name  # f"{ros_type}::{param_name}"

        special_types = {"duration", "time"}

        # In the case that we have a duration or time primitive,
        # we are going to adjust to ask for the secs and nsecs required for each type.
        if ros_type in special_types:
            type_obj['children'] = [
                {'name': 'uint32 secs', 'children': []},
                {'name': 'uint32 nsecs', 'children': []}
            ]

        if not len(type_obj["children"]):
            return {"name": p_name,
                    "type": ros_type, "ros_type": ros_type}

        sub_types = []

        for child in type_obj["children"]:
            sub_types.append(self._convert_type(child, p_name))

        return {"name": p_name, "type": sub_types, "ros_type": ros_type}
