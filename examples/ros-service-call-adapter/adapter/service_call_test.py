import rosservice
import rospy
import utils

rospy.wait_for_service("/random")
s = rospy.ServiceProxy("/random", utils.get_service_type_obj("/random"))

import pdb
pdb.set_trace()