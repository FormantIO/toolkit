import logging

import rosgraph
import roslib
import rospy
import rostopic

logger = logging.getLogger()


def get_topics():
    """
    Return a list of all currently published topics along with
    the topic types. 
    Will omit all topics specified in topic_ignore.json
    """
    return [topic for topic, _ in rospy.get_published_topics()]


def is_valid_ros_topic(topic):
    """Returns boolean indicating that the passed ROS topic can be used."""
    if not rosgraph.is_master_online():
        return False
    try:
        data_type = rostopic.get_topic_type(topic, blocking=False)[0]
    except rostopic.ROSTopicIOException:
        logger.info(
            "Cannot validate topic: %s, ROS master could not be reached." % topic
        )
        return False
    if not data_type:
        return False  # the topic doesn't exist

    message_class = roslib.message.get_message_class(data_type)
    if not message_class:
        logger.warning(
            "Cannot subscribe to %s with type %s;" % (topic, data_type)
            + " no message class found."
            + " catkin workspace may be unsourced.",
        )
        return False

    message_description = message_class._full_text
    if not message_description:
        logger.warning(
            "Cannot subscribe to %s; no message description found." % topic,
        )
        return False

    return True


def get_topic_type_obj(topic):
    """Return the type obj of a message for a given topic"""
    data_type = rostopic.get_topic_type(topic, blocking=False)[0]
    return roslib.message.get_message_class(data_type)


def get_ros_type_obj(
    type # type: str
):
    return roslib.message.get_message_class(type)
