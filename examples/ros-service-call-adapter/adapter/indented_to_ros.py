

def convert_to_ros(indented_json_format):
    converter = IndentedToRos(indented_json_format)
    return converter.get_converted()

class IndentedToRos:

    def __init__(self, indented_format):
        self._indented_format = indented_format

        self._ros_format = []
        self._convert()
    
    def get_converted(self):
        return self._ros_format

    def _convert(self):
        
        for param in self._indented_format:
            if "---" in param["name"]:
                break 
            self._ros_format.append(self._convert_type(param))

    def _convert_type(self, type_obj, parent_name=""):
        
        
        ros_type, param_name = type_obj["name"].lstrip().rstrip().replace("\n","").split(" ")
        p_name = f"{parent_name}.{param_name}" if len(parent_name) else param_name
      

        if not len(type_obj["children"]):
            return {"name":p_name,
                    "type":ros_type}
        
        sub_types = []
        
        for child in type_obj["children"]:
            sub_types.append(self._convert_type(child, p_name))
        
        return {"name":p_name, "type":sub_types}
