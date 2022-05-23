
from typing import List
from formant.sdk.agent.v1.client import Client as FormantClient
from config import Config
from logger import getLogger
import time

logger = getLogger()

class Adapter:

    def __init__(self):
        
        self._fclient = FormantClient()
        self._services = {}

        self._config = Config().get_config()
        self._button_map = self._config["button-mapping"]

    def run(self):
        
        self._fclient.register_teleop_callback(self._handle_button_press, ["Buttons"])
        self._fclient.register_command_request_callback(self._handle_command, self._config["service-commands"])

        while True:
            time.sleep(1)

    def _handle_button_press(self, button_press):
        button_name = button_press.bitset.bits[0].key
        if button_name not in self._button_map:
            logger.info(f"Button {button_name} has not service mapping")
        self._handle_service_call(button_name, [])

    def _handle_command(self, data):
        data.text()
        
        import pdb
        pdb.set_trace()

    def _handle_service_call(self, service_name: str, service_args: List[str]):
        pass