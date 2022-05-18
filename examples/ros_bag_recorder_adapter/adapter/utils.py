from typing import Optional
from matplotlib.pyplot import get
import rostopic
import rospy
import rosbag
import rosgraph
import roslib
import datetime
import logging
from json import load as jsonload
from globals import formant_client
from datetime import datetime


def get_topics():
    """
    Return a list of all currently published topics along with
    the topic types. 

    Will omit all topics specified in topic_ignore.json
    """
    return [topic for topic, _ in rospy.get_published_topics()]


def generate_bagname():
    """Create a bagname from the config parameters."""

    name_string = get_config_variable("bag_naming_convention")
    date_time_format = get_config_variable("date_time_string")
    file_location = get_config_variable("bag_storage_path")
    date_time_string =  datetime.now().strftime(date_time_format)
    bag_num = generate_bagname.bag_num
    generate_bagname.bag_num += 1

    name_string = name_string.replace("$bn", str(bag_num))
    name_string = name_string.replace("$dt", date_time_string)

    name_string = file_location + "/" + name_string

    return name_string

generate_bagname.bag_num = 0

def generate_bag(name=None):
    """
    Create a new rosbag. 
    If name is None, then the name will be automatically generated.

    If name is None, then append_time will default to false. 
    """

    if name is None:
        name = generate_bagname()

    return rosbag.Bag(name, mode="w")


def load_config():
    """Load the config.json file into memory and return"""

    if hasattr(load_config, 'config'):
        return load_config.config

    with open('config.json') as f:
        config = jsonload(f)

    load_config.config = config
    return config


def get_configuration_param(param_name: str, strict=False):
    """
    This function fetches a configuration parameter. 
    It first looks at key-value pairs for a key with the named configuration.
    If the key does not exist, then it defaults to using config.json. 

    If strict is True and a param_val is not found as a 
    key-value pair, then None is returned. 
    """
    param_val = formant_client.get_app_config(param_name, None)

    if param_val is not None:
        return param_val

    if strict:
        return None

    return load_config()[param_name]


def is_valid_ros_topic(topic):
    """Returns boolean indicating that the passed ROS topic can be used."""
    if not rosgraph.is_master_online():
        return False
    try:
        data_type = rostopic.get_topic_type(topic, blocking=False)[0]
    except rostopic.ROSTopicIOException:
        logging.info(
            "Cannot validate topic: %s, ROS master could not be reached." % topic
        )
        return False
    if not data_type:
        return False  # the topic doesn't exist

    message_class = roslib.message.get_message_class(data_type)
    if not message_class:
        logging.warning(
            "Cannot subscribe to %s with type %s;" % (topic, data_type)
            + " no message class found."
            + " catkin workspace may be unsourced.",
        )
        return False

    message_description = message_class._full_text
    if not message_description:
        logging.warning(
            "Cannot subscribe to %s; no message description found." % topic,
        )
        return False

    return True


def get_topic_type_obj(topic):
    """Return the type obj of a message for a given topic"""
    data_type = rostopic.get_topic_type(topic, blocking=False)[0]
    return roslib.message.get_message_class(data_type)


def load_configuration_variables():
    """
    Load configuration variables into a dictionary. 
    """

    if hasattr(load_configuration_variables, "config_values"):
        return load_configuration_variables.config_values

    config_vars = [
        ("subscribe_to_all", "required"),
        ("topics", "required"),
        ("bag_length", "required"),
        ("bag_overlap", "required"),
        ("bag_storage_path", "optional", "bags/"),
        ("bag_naming_convention", "optional", "%bn_%dt"),
        ("date_time_string", "optional", "%d_%m_%Y-%H_%M_%S"),
        ("ignore_topics", "optional", []),
        ("loglevel", "optional", "WARN"),
        ("topic_refresh_rate", "optional", 2)
    ]

    config_values = {}

    json_config = load_config()

    for config_var in config_vars:
        config_val = get_configuration_param(config_var[0], strict=True)

        if config_val is None and config_var[0] in json_config:
            config_val = get_configuration_param(config_var[0])

        elif config_val is None and config_var[0] not in json_config:
            if config_var[1] == "required":
                logging.fatal(f"Required config variable {config_var[0]}\
                    not found as a key-value pair or in config.json. Exiting.")
                exit(1)
            if config_var[1] == "optional":
                logging.warning(f"Config variable {config_var[0]} not \
                    found as a key-value pair or in config.json. Using \
                        default value: {config_var[2]}")
                config_val = config_var[2]

        config_values[config_var[0]] = config_val

        load_configuration_variables.config_values = config_values
    return config_values


def get_config_variable(variable: str):
    """
    Get a specified config variable.
    """

    config_vars = load_configuration_variables()

    if variable not in config_vars:
        logging.fatal(f"Requested config variable {variable} does not exist.")
        exit(1)

    return config_vars[variable]

def get_log_level():
    log_level = get_config_variable("loglevel")
    if log_level.upper() == "DEBUG":
        return logging.DEBUG
    elif log_level.upper() == "INFO":
        return logging.INFO
    elif log_level.upper() == "WARN":
        return logging.WARN
    elif log_level.upper() == "CRITICAL":
        return logging.CRITICAL

logging.basicConfig(level=get_log_level())