#!/usr/bin/python
"""
This file contains the definition of the Adapter itself. The adapter is what allows us to start up the bag recorder
"""

import logging
import threading
import time

from baghandler import BagHandler
from config import Config
from genpy import Duration
from utils import *

from formant.sdk.agent.v1.client import Client as FormantClient

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
        self._recording = False
        self._recording_thread = None
        self._fclient = FormantClient() 

        self.bag_thread.start()

    def handle_command(self, message):
        
        if message.command == "start_ros_recorder":
            if self._recording:
                return
            
            self._recording_thread = threading.Thread(target=self.start_recording, args=(None, True))
            self._recording_thread.start() 

        if message.command == "stop_ros_recorder": 
            self._recording = False
            
        if message.command == "start_ros_recorder_duration":
            duration = rospy.Duration(secs=float(message.text))
            self.start_recording(duration, True) 

    def run(self):
        rospy.init_node("rospy_bag_recorder")
        rospy.on_shutdown(self.shutdown)

        self._fclient.register_command_request_callback(self.handle_command, 
            ["start_ros_recorder", "stop_ros_recorder", "start_ros_recorder_duration"])
        
        rospy.spin() 

    def setup_topics(self):
        ignore_topics = set(self.config.get_param("ignore_topics"))

        if self.config.get_param("subscribe_to_all"):
            self.topics = get_topics()

        for topic in self.topics:
            
            if topic in ignore_topics or topic in self.subscriptions:
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

    def start_recording(self, duration=None, refresh_topics=False):
        self.setup_topics()
        
        topic_refresh_rate = int(self.config.get_param("topic_refresh_rate"))
        last_update = rospy.Time.now() 

        if(duration is None):
            duration = rospy.Duration(secs=9999999)

        start_time = rospy.Time.now() 
        self._recording = True
        while(self._recording and not self.is_shutdown() and rospy.Time.now() - start_time < duration):
            rospy.sleep(0.2)
            if(rospy.Time.now() - last_update > rospy.Duration(secs=topic_refresh_rate) and refresh_topics):
                self.setup_topics() 
                last_update = rospy.Time.now()
        self._recording = False
        self.stop_recording() 

    def stop_recording(self):
        
        for sub in self.subscriptions.items():
            sub[1].unregister()
        self.subscriptions = {} 

    def shutdown(self):
        for sub in self.subscriptions.items():
            sub[1].unregister()
            logger.info(f"Successfully unsubscribed from {sub[0]}")
        self._shutdown_system = True
        self.bag_thread.join()
        self.bag_handler.shutdown()

    def is_shutdown(self):
        return self._shutdown_system
