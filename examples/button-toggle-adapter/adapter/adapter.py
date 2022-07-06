import time

import rospy
from formant.sdk.agent.v1.client import Client as FormantClient

from buttontoggle import ButtonHandler


class Adapter:

    def __init__(self):
        self._button_handler = ButtonHandler()
        self._is_shutdown = False

    def start(self):
        while(not rospy.is_shutdown()):
            time.sleep(1)
        self.shutdown()

