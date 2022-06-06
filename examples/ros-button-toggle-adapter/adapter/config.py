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