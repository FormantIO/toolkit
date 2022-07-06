"""This file contains the definition of a rosbag"""

import os
from datetime import datetime

import rosbag

from config import Config


class BagFactory:

    def __init__(self):
        """Initialize the bag factory"""

        config = Config() 

        self.num_bags = 0
        self.naming_convention = config.get_param("bag_naming_convention")
        self.dt_convention = config.get_param("date_time_string")
        self.base_path = config.get_param("bag_storage_path")

    def create_bag(self):
        """Generate an unopened RosBag object with a .active extension"""

        base_path = self.base_path.rstrip().lstrip()
        base_path = base_path[:-1] if base_path[-1] == "/" else base_path

        name = base_path + "/" + self._get_bag_name()
        bag = RosBag(name) 
        self.num_bags += 1
        return bag
    
    def _get_bag_name(self):
        """Return the name of the next bag to be created."""
        bag_name = self.naming_convention.replace("$bn", str(self.num_bags))
        bag_name = bag_name.replace("$dt", datetime.now().strftime(self.dt_convention)) 
        bag_name = bag_name + ".active"
        return bag_name

class RosBag:
    
    def __init__(self, name: str):
        
        self.config = Config()
        
        self.bag_path = self.config.get_param("bag_storage_path")

        self.name = name
        self.bag = None

        self._is_open = False

    def open(self):
        """Open the bag in memory as a *.bag.active file"""
        print("Opening new bag")
        self.bag = rosbag.Bag(self.name, 'w') 
        self._is_open = True

    def close(self):
        """Closes the bag, removing the .active suffix"""
        print("Closing Bag")
    
        if not self.is_open():
            return

        self.bag.close()
        self._is_open = False

        # -7 is the length of the .active suffix which we are removing.
        new_name = self.name[:-7]
        
        os.system(f"mv {self.name} {new_name}")

    def write(self, message, topic):
        """Writes the specified message to the bag"""
        self.bag.write(topic, message)

    def is_open(self):
        return self._is_open