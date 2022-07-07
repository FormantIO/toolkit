
import logging
import time
from typing import List

import rospy
from formant.sdk.agent.v1.client import Client as FormantClient

from config import Config
from input_to_ros_service_params import parse
from services import ServiceChecker
from utils import service_call

logger = logging.getLogger()


class Adapter:

    def __init__(self):
        """Initialize the adapter"""

        self._fclient = FormantClient()
        self._config = Config().get_config()
        self._button_map = self._config["button-mapping"]
        self._service_checker = ServiceChecker()

    def run(self):
        """Run the adapter. This function will never return, but is non-blocking."""

        self._service_checker.start()

        self._fclient.register_teleop_callback(
            self._handle_button_press, ["Buttons"])
        self._fclient.register_command_request_callback(
            self._handle_command, self._config["service-commands"])

        logger.info(
            "Callback's have successfully been registered with the Formant client.")

        while True:
            time.sleep(1)

    def _handle_button_press(self, button_press):
        """Handle a button press along with the proper parameters."""

        if not button_press.bitset.bits[0].value:
            return

        button_name = button_press.bitset.bits[0].key

        logger.info(f"Button press received from button {button_name}")

        if button_name not in self._button_map:
            logger.info(f"Button {button_name} has not service mapping")

        service_name = self._button_map[button_name][0]
        params = self._button_map[button_name][1]

        self._handle_service_call(service_name, params)

    def _handle_command(self, data):
        """Handles an incoming formant command as specified in service-commands in config.json"""

        logger.info(
            f"New command received. Parsing and executing: {data.text}")

        print(f"New command received. Parsing and executing: {data.text}")

        try:
            datum = parse(data.text)
        except Exception as e:
            logger.warn(f"Failed parsing {data.text}. Dropping command. Reason: {e}")
            return

        service_name = datum[0]
        service_args = datum[1]
 
        self._handle_service_call(service_name, service_args)

    def _handle_service_call(self, service_name: str, service_args: List[str]):
        """This allows the adapter to call a ROS service, then post the response."""

        logger.info(
            f"Sending service call to service {rospy.resolve_name(service_name)}")
        response = service_call(
            rospy.resolve_name(service_name), *service_args)

        if response and len(str(response)):
            self._post_service_data(service_name, str(response))

    def _post_service_data(self, service_name: str, data: str):
        """Post's a string to Formant given the service_name string."""
        # stream_name = rospy.resolve_name(service_name).replace("/", ".")
        # stream_name = stream_name[1:] if stream_name[0] == "." else stream_name
        response_stream = f"ros.services.response"
        try:
            self._fclient.post_text(response_stream, str(data))
        except Exception:
            logger.info("Failed to post response")
            return

        logger.info(
            f"Successfully send '{str(data)}' to Formant stream '{response_stream}'")

    def shutdown(self):
        self._service_checker.shutdown()
