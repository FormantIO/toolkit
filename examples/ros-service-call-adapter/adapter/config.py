
from json import load as json_load
import json
from json_schema_validator import JsonSchemaValidator
from formant.sdk.agent.v1.client import Client as AgentClient
class Config:
    """The config class loads the config.json from memory."""

    def callabck(self, config):
        self.config_raw = config

    def __init__(self):

        agentClient = AgentClient()

        JsonSchemaValidator(
            agentClient,
            "Ros Service Call Adapter",
            self.callback,
            True,
            False,
        )

        self._config = {}
        self._config["service-command"] = ["rosservice"]
        self._config["api-button-mapping"] = {}
        self._config["ros-button-mapping"] = {}

        for ros_button_config in self.config_raw["ros-button-mapping"]:
            topic = ros_button_config["topic"]
            button_config_raw = ros_button_config["button-config"]

            try:
                button_config_json = json.loads(button_config_raw)
            except json.decoder.JSONDecodeError:
                continue

            self._config["ros-button-mapping"][topic] = button_config_json

        for api_button_config in self.config_raw["api-button-mapping"]:
            button_name = api_button_config["api_button_name"]
            button_config_raw = api_button_config["button-config"]

            try:
                button_config_json = json.loads(button_config_raw)
            except json.decoder.JSONDecodeError:
                continue

            self._config["api-button-mapping"][button_name] = button_config_json

    def get_config(self):
        return self._config
