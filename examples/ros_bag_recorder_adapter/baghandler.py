#!/usr/bin/python
"""
This file contains the definition of the BagHandler which is used to control what bags are opened along with 
messages get written to what bag. 
"""

from utils import *
import time
import datetime
from queue import Queue


class BagHandler:
    """
    The BagHandler handles management of ROS bags and provides 
    an interface that allows messages to be queued for writing.
    """

    def __init__(self):
        """Initialize the bag handler"""

        # The elements in the message queue are of the form
        # (timestamp: Datetime Obj, message)
        self.message_queue = Queue()

        self.bag_length = get_config_variable("bag_length")
        self.bag_overlap = get_config_variable("bag_overlap")

        # Run check for valid bag lengths
        if(self.bag_overlap > self.bag_length / 2):
            logging.critical("Error: bag overlap is too large. \
                Overlap must be <= bag_length / 2. Exiting.")
            exit(1)

        # Bag 1. This is always the newest bag
        self.bag1 = generate_bag()
        self.bag1_start = datetime.datetime.now()
        self.bag1_end = datetime.datetime.now() +\
            datetime.timedelta(seconds=self.bag_overlap)

        # Bag 2. This bag may or may not exist. It depends on the
        # Size of the overlap.
        self.bag2 = generate_bag()
        self.bag2_start = datetime.datetime.now()
        self.bag2_end = datetime.datetime.now() +\
            datetime.timedelta(seconds=self.bag_length)

        self.bag_index = 2

        self.last_message_time = datetime.datetime.now()

    def enqueue_message(self, message):
        """Give the bag handler a new message."""

        # TODO: check if I can get the time out of the message rather
        #       than using datetime.datetime.now()
        self.message_queue.put((datetime.datetime.now(), message))

    def run(self, shutdown):
        """
        The run method so the BagHandler can be run in its own thread
        """
        while(not shutdown()):
            if self.message_queue.empty():
                time.sleep(5/10)
                continue

            self._bag_check()

            timestamp, message = self.front_item  # = self.message_queue.get()

            # If the messages are out of order, then we just drop the old message
            if timestamp < self.last_message_time:
                logging.info("Bag Error: dropping message. Out of order error")
                continue
            self.last_message_time = timestamp

            # We can always write to bag1 as we ran the bag check method
            BagHandler._write_to_bag(self.bag1, message)

            if timestamp >= self.bag2_start:
                BagHandler._write_to_bag(self.bag2, message)
        
        self._close_bags()

    @staticmethod
    def _write_to_bag(bag: rosbag.Bag, message):
        """
        Write a message to a bag
        """
        data, topic_name = message
        bag.write(topic_name, data)

    def _bag_check(self):
        """
        This method checks to see if the bags are valid for the timestamp that is at
        the start of the queue. 

        If it finds that it needs a new bag or does not need a bag, then this method
        will update bag1 and bag2 
        """

        # TODO : I'd like this to not remove the front item by not using a get()
        timestamp, item = self.message_queue.get() 
        self.front_item = (timestamp, item)

        # There is no longer a need for bag1
        if timestamp > self.bag1_end:
            self.bag1.close()
            self.bag1 = self.bag2
            self.bag1_start = self.bag2_start
            self.bag1_end = self.bag2_end

            self.bag2 = generate_bag()
            logging.debug(f"Created new bag. Bag index {self.bag_index}")

            self.bag_index += 1
            self.bag2_start = self.bag1_end - \
                datetime.timedelta(seconds=self.bag_overlap)
            self.bag2_end = self.bag2_start + \
                datetime.timedelta(seconds=self.bag_length)

            # We have to recurse because there is a chance that the new bag is
            # Not new enough and we need to generate an even newer bag
            self._bag_check()

    def _close_bags(self):
        """Close the bag handlers bags."""
        self.bag1.close()
        self.bag2.close()
