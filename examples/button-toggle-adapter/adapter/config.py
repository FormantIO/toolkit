import json


class Config:
    """The config class gets the config.json file and loads it into memory."""

    def __init__(self):
        """Initialize the Config class"""
        with open("config.json", "r") as f:
            self._config = json.load(f)

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
