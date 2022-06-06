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
    
    def callback_settings(self, type="global", name=None):
        """Return the global callback settings or specific callback settings."""
        if type == "global":
            return ButtonConfiguration(
                "global",
                self._config["global_configuration"]
            )

class ButtonConfiguration:
    def __init__(self, name: str, configuration):
        """
        Initialize the button configuration.
        
        @param name - Either Button name or topic name
        @param configuration - The config as seen in the JSON
        """
        self._name = name
        self._configuration = configuration

    def get_name(self):
        return self._name
    
    def get_export_topic(self):
        return self._configuration["Export Topic"]

    def publish_on_change(self):
        "Return True if the 'on change' setting is set"
        return self._check_callback_config("on change")
    
    def publish_on_pulse(self):
        "Return True if the 'pulse' setting is set"
        return self._check_callback_config("pulse")
    
    def publish_on_subscriber(self):
        "Return True if the 'on subscriber' setting is set"
        return self._check_callback_config("on subscriber")

    def _check_callback_config(self, setting):
        return setting in self._configuration["Callback Settings"]

class CallbackSettings:
    def __init__(self, configuration):
        pass 

    