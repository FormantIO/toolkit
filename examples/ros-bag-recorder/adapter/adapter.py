#!/usr/bin/python
"""
This file contains the definition of the Adapter itself. The adapter is what allows us to start up the bag recorder
"""

import logging
import threading
import time

from baghandler import BagHandler
from config import Config
from utils import *

# logger.basicConfig(level=get_log_level())

logger = logging.getLogger()


class Adapter:
    """
    The adapter class that will run and control the functionality
    """

    def __init__(self):

        self.config = Config()

        # Set the topics and types for the specified adapter
        if self.config.get_param("subscribe_to_all"):
            self.topics = get_topics()
        else:
            self.topics = self.config.get_param("topics")

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

        ignore_topics = set(self.config.get_param("ignore_topics"))

        for topic in self.topics:
            
            if topic in ignore_topics:
                continue

            if not is_valid_ros_topic(topic):
                logger.warning(f"Topic f{topic} cannot be subscribed to by "\
                    +"the bag recorder. Skipping topic.")
                continue

            
            sub = rospy.Subscriber(topic, get_topic_type_obj(topic),
                                   self.bag_handler.message_callback,
                                   topic)

            self.subscriptions[topic] = sub

            logger.info(f"Successfully subscribed to {topic}")
    
        rospy.on_shutdown(self.shutdown)
        
        topic_refresh_rate = int(self.config.get_param("topic_refresh_rate"))

        if not self.config.get_param("subscribe_to_all"):
            rospy.spin()

        else:
            while(not self._shutdown_system):
                time.sleep(topic_refresh_rate)
                
                self.topics = get_topics()

                for topic in self.topics:
                    if topic in self.subscriptions or topic in ignore_topics:
                        continue 
                
                    logger.info(f"Found New Topic: {topic}")
                
                    sub = rospy.Subscriber(topic, get_topic_type_obj(topic),
                                   self.bag_handler.message_callback,
                                   topic)
                    self.subscriptions[topic] = sub
                
                    logger.info(f"Successfully subscribed to new topic: {topic}")


    def shutdown(self):
        for sub in self.subscriptions.items():
            sub[1].unregister()
            logger.info(f"Successfully unsubscribed from {sub[0]}")
        self._shutdown_system = True
        self.bag_thread.join()

    def is_shutdown(self):
        return self._shutdown_system
