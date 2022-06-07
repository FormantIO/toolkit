
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
            if "---" in param["name"]: # Signifies the end of the request part of a ROS message
                break 
            self._ros_format.append(self._convert_type(param))

    def _convert_type(self, type_obj, parent_name=""):
        ros_type, param_name = type_obj["name"].lstrip().rstrip().replace("\n","").split(" ")
        p_name = f"{parent_name}.{param_name}" if len(parent_name) else param_name#f"{ros_type}::{param_name}"
      

        if not len(type_obj["children"]):
            return {"name":p_name,
                    "type":ros_type}
        
        sub_types = []
        
        for child in type_obj["children"]:
            sub_types.append(self._convert_type(child, p_name))
        
        return {"name":p_name, "type":sub_types}
