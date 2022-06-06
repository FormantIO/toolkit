from rospy import Service
from services import ServiceChecker

checker = ServiceChecker()
checker._check_services()

print(checker._services_json) 