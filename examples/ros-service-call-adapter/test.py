import rosservice
import rostopic
import roslib

import genpy

print(rosservice.get_service_list())

def get_topic_type_obj(topic):
    """Return the type obj of a message for a given topic"""
    data_type = rostopic.get_topic_type(topic, blocking=False)[0]
    return roslib.message.get_message_class(data_type)

t2 = get_topic_type_obj("/sample")

t = rosservice.get_service_type("/add_two_ints")

typ = genpy.message._get_message_or_service_class('srv', t)
import pdb
pdb.set_trace()

# s_type = rosservice.get_service_class_by_name("/add_two_ints")
# import pdb
# pdb.set_trace()
