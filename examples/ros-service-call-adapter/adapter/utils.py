
import rosservice

def current_services():
    """Return the currently active list of services."""
    return set(rosservice.get_service_list())