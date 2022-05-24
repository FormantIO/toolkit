
from json import load as json_load


class Config:
    """The config class loads the config.json from memory."""

    def __init__(self):
        with open("config.json", "r") as f:
            self.config_json = json_load(f)

    def get_config(self):
        return self.config_json
