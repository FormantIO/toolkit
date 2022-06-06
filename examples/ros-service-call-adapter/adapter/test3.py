import rosmsg
import rosservice

from indented_parser import parse_indented_string
from indented_to_ros import convert_to_ros 

import json

s = rosmsg.get_srv_text(rosservice.get_service_type("/random"))

print(s)

pi = parse_indented_string(s) 
c = convert_to_ros(pi) 

with open('ros_output.json', 'w') as f:
    json.dump( c, f )