import json
import time
import json_schema_validator
from formant.sdk.agent.v1.client import Client as AgentClient

class Config:
    """The config class gets the config.json file and loads it into memory."""

    def callback(self, config):
        self.config_raw = config

    def __init__(self):
        """Initialize the Config class"""
        self.config_raw = {}
        
        agentClient = AgentClient()

        json_schema_validator.JsonSchemaValidator(
            agentClient,
            "button-toggle-adapter",
            self.callback,
            True,
            True,
        )

        while len(self.config_raw) == 0:
            time.sleep(0.1)

        self._config = {}
        self._config["ROS-buttons"] = {}
        self._config["API-buttons"] = {}

        for ros_button_config in self.config_raw["ros-buttons"]:
            input_topic = ros_button_config["input_topic"]
            output_topic = ros_button_config["output_topic"]
            self._config["ROS-buttons"][input_topic] = {
                "input_topic": input_topic,
                "output_topic": output_topic,
            }
        
        for api_button_config in self.config_raw["api-buttons"]:
            name = api_button_config["api_button_name"]
            output_topic = api_button_config["output_topic"]
            self._config["API-buttons"][name] = {
                "name": name,
                "output_topic": output_topic,
            }
        
        self._config["global-configuration"] = {
            "initial-state": self.config_raw.get("default-button-state", False),
            "publish-on-subscription": self.config_raw.get("publish-on-subscription", False),
        }
        
        print("Using config:", self._config)

    def get_config(self):
        """Get the loaded config."""
        return self._config

    def get_global_config(self):
        return self._config["global-configuration"]

    def get_button_config(self, type, name):
        """Return the global callback settings or specific callback settings."""
        return ButtonConfiguration(
            self.get_config()[type][name],
            self.get_global_config()
        )


class ButtonConfiguration:
    def __init__(self, configuration, global_config):
        """
        Initialize the button configuration.

        @param name - Either Button name or topic name
        @param configuration - The config as seen in the JSON
        """
        self._configuration = configuration
        self._global_configuration = global_config

    def get(self, param, default=None):
        """
        Return the value of param if it exists in the 
        button config. If not, return global config.
        If that does not exist, then return the default value.
        """
        if param in self._configuration:
            return self._configuration[param]
        return self._global_configuration.get(param, default)
