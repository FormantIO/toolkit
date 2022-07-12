
import ast
import logging
from distutils.util import strtobool
from json import load as jsonload
from threading import Lock
from typing import List

from formant.sdk.agent.v1 import Client as FormantClient

logger = logging.getLogger(__name__)


class Config:
    """Provides access to all of the adapters configurations."""

    _configured = False
    _config_lock = Lock()

    def __init__(self):
        """Initializer for the Config class."""

        # Use a singleton like pattern to only need 1 formant client
        # And 1 dictionary to store all configuration variables.
        if not Config._configured:
            Config._config_lock.acquire()

            # May have happened while acquiring the Lock,
            # So we check twice.
            if not Config._configured:
                Config._formant_client = FormantClient()
                Config._config_values = Config._load_config_vars()
                Config._configured = True
                # del Config._formant_client

            Config._config_lock.release()

    @staticmethod
    def get_param(name: str):
        """Return a config variable."""

        types = {
            "subscribe_to_all": bool,
            "topics": List,
            "ignore_topics": List,
            "topic_refresh_rate": int,
            "bag_length": int,
            "bag_overlap": int,
            "bag_storage_path": str,
            "bag_naming_convention": str,
            "date_time_string": str,
            "loglevel": str
        }

        value = Config._config_values[name] 

        if isinstance(value, types[name]):
            return value

        elif types[name] == str:
            return str(value) 
        
        elif types[name] == bool:
            if isinstance(value, str):
                if value == "1":
                    return True
                if value == "0":
                    return False
                return bool(strtobool(value)) 
            return bool(value) 
        
        elif types[name] == List and isinstance(value, str):
            return ast.literal_eval(value)

        elif types[name] == int:
            return int(value) 

        logger.fatal(f"Cannot parse parameter {name} with value {value}. Exiting.")
        exit(1)

    @staticmethod
    def _get_config_var_formant(param_name: str):
        """
        Return the config param associated with param_name 
        from the formant agent. If it does not exist, then return None
        """
        return Config._formant_client.get_app_config(param_name, None)

    @staticmethod
    def _load_config_vars():
        """
        Load configuration variables into a dictionary. 
        """

        config_vars = [
            ("subscribe_to_all", "required"),
            ("topics", "required"),
            ("bag_length", "required"),
            ("bag_overlap", "required"),
            ("bag_storage_path", "optional", "bags/"),
            ("bag_naming_convention", "optional", "bag%bn_%dt.bag"),
            ("date_time_string", "optional", "%d_%m_%Y-%H_%M_%S"),
            ("ignore_topics", "optional", {}),
            ("loglevel", "optional", "WARN"),
            ("topic_refresh_rate", "optional", 2)
        ]

        config_values = {}

        json_config = Config._load_config_json()

        for config_var in config_vars:
            config_val = Config._get_config_var_formant(config_var[0])

            if config_val is None and config_var[0] in json_config:
                config_val = json_config[config_var[0]]

            elif config_val is None and config_var[0] not in json_config:
                if config_var[1] == "required":
                    logger.fatal(f"Required config variable {config_var[0]}\
                        not found as a key-value pair or in config.json. Exiting.")
                    exit(1)
                if config_var[1] == "optional":
                    logger.warning(f"Config variable {config_var[0]} not \
                        found as a key-value pair or in config.json. Using \
                            default value: {config_var[2]}")
                    config_val = config_var[2]

            config_values[config_var[0]] = config_val
            logger.debug(f"Loaded config var: {config_var[0]} with value: {config_val}")

        return config_values

    @staticmethod
    def _load_config_json():
        """Load the config.json file."""

        with open('config.json') as f:
            config = jsonload(f)

        return config
