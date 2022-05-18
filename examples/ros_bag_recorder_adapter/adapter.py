#!/usr/bin/python
"""
This file contains the definition of the Adapter itself. The adapter is what allows us to start up the bag recorder
"""

import threading
import logging
from utils import *
from typing import Tuple
from baghandler import BagHandler


def save_msg_to_bag(data, callback_params: Tuple[BagHandler, str, str]):
    """
    This is the callback that is used for all incoming messages
    """
    # Unwrap the extra call params here
    bag_handler, topic_name = callback_params
    
    # Put the message into the queue with current timestamp
    bag_handler.enqueue_message((data, topic_name))


class Adapter:
    """
    The adapter class that will run and control the functionality
    """

    def __init__(self):

        # Set the topics and types for the specified adapter
        if get_config_variable("subscribe_to_all"):
            self.topics = get_topics()
        else:
            self.topics = get_config_variable("topics")

        # Create a new rosbag with the name as the time and date
        self.bag_handler = BagHandler()
        self.bag_thread = threading.Thread(target=self.bag_handler.run, args=(self.is_shutdown,))
        self.subscriptions = {}
        self._shutdown_system = False

        self.bag_thread.start()

    def run(self):
        """
        Start all the callbacks and their specified topics
        """
        rospy.init_node("ros_bag_recorder_adapter")
        for topic in self.topics:
            if not is_valid_ros_topic(topic):
                logging.warning(f"Topic f{topic} cannot be subscribed to by \
                    the bag recorder. Skipping topic.")
                continue

            
            sub = rospy.Subscriber(topic, get_topic_type_obj(topic),
                                   save_msg_to_bag,
                                   (self.bag_handler, topic))

            self.subscriptions[topic] = sub

            logging.info(f"Successfully subscribed to {topic}")
            print(f"Successfully subscribed to {topic}")

    def close(self):
        for sub in self.subscriptions:
            sub.unregister()

    def shutdown(self):
        for sub in self.subscriptions.items():
            sub[1].unregister()
            logging.info(f"Successfully unsubscribed from {sub[0]}")
        self._shutdown_system = True
        self.bag_thread.join()

    def is_shutdown(self):
        return self._shutdown_system