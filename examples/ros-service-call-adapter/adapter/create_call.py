import services
import json
import rospy 

def main():
    s_checker = services.ServiceChecker()

    current_services = json.loads(s_checker.get_services_json())

    print("Current services: \n")
    for service in current_services:
        print(f"\t{service}") 
    
    print("\n")

    has_service = False
    while(not has_service):
        service = rospy.resolve_name(input("Please enter service: "))
        if service not in current_services:
            print("Service does not exist.. ")
            continue
        has_service = True
    
    call_params = CallCreator(current_services[service])

    call_full =  json.dumps({service:call_params.get_parsed()})
    
    print("Successfully Generated service call...\n\n")
    
    print(json.dumps(json.loads(call_full)))
    print()

class CallCreator:

    def __init__(self, call_desc):
        self._call = {} 
        self._call_desc = call_desc
        self.parsed = self.parse()

    def get_parsed(self):
        return self.parsed

    def parse(self):
        return self._parse_object(self._call_desc)

    def _parse_object(self, obj):
        values = {}

        properties_dict = obj["properties"]

        for param in properties_dict:
            if CallCreator.is_primitive(properties_dict[param]):
                values[param] = self._parse_primitive(properties_dict[param])
            
            elif CallCreator.is_array(properties_dict[param]):
                values[param] = self._parse_array(properties_dict[param])

            else:
                values[param] = self._parse_object(properties_dict[param])
        
        return values

    def _parse_primitive(self, obj):
        if obj["type"] == "number":
            return float(input(f"Please enter a number for parameter '{obj['title']}': "))

        if obj["type"] == "string":
            return input(f"Please enter a value for parameter '{obj['title']}': ")

        if obj["type"] == "integer":
            return int(input(f"Please enter an integer for parameter '{obj['title']}': "))
        
        if obj["type"] == "boolean":
            i = input(f"Please enter a boolean ('true' or 'false') for parameter '{obj['title']}': ")
            if i.lower() == "true":
                return True
            elif i.lower() == "false":
                return False
            else:
                return False

    def _parse_array(self, obj):
        if obj["items"]["type"] == "string":
            t = "values"
        else:
            t = "numbers"
        
        values = input(f"Please enter a comma separated array of {t} for parameter '{obj['title']}':\n\t")
        values_split = [v.rstrip().lstrip() for v in values.split(",")]

        if t == "numbers":
            return [float(n) for n in values_split]
        
        return values_split

    @staticmethod
    def is_primitive(desc):
        if desc["type"] in {"string", "number", "integer", "boolean"}:
            return True
        return False
    
    @staticmethod
    def is_array(desc):
        if desc["type"] == "array":
            return True
        return False

if __name__ == "__main__":
    main()
