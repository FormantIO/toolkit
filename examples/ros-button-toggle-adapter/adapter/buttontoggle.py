
import rospy
from formant.sdk.agent.v1.client import Client as FormantClient
from config import Config

class ButtonHandler:

    def __init__(self):
        
        self._fclient = FormantClient()
        self._fclient.register_teleop_callback(
            self._handle_button_press, ["Buttons"])

        self._config = Config().get_config() 

    def _handle_button_press(self, button_press):
        import pdb
        pdb.set_trace()

    

class ButtonToggler:
    """The button toggler class takes a boolean toggle class and creates a button toggler."""

    def __init__(self, topic):
        """"""
        pass

    def _handle_state_change(self):
        pass

    def set_button_state(self, state: bool):
        pass