
import rospy
from formant.sdk.agent.v1.client import Client as FormantClient
from std_msgs.msg import Bool as RosBool

from config import ButtonConfiguration, Config


class ButtonHandler:
    
    def __init__(self):

        self._fclient = FormantClient()

        self._config = Config()

        rospy.init_node("button_toggle_adapter")
        self._subscriptions = {}

        self._buttons = {}

        self._init_API_buttons()
        self._init_ROS_buttons()

    def _init_API_buttons(self):
        self._fclient.register_teleop_callback(
            self._API_button_callback, ["Buttons"])

        API_config = self._config.get_config()["API-buttons"]

        for button in API_config:
            self._buttons[ButtonHandler.get_API_button_name(button)] = Button(
                self._config.get_button_config("API-buttons", button)
            )

    def _init_ROS_buttons(self):
        ros_button_topics = self._config.get_config()["ROS-buttons"]

        for button_topic in ros_button_topics:
            try:
                sub = rospy.Subscriber(
                    button_topic,
                    RosBool,
                    self._ros_topic_button_callback,
                    button_topic)
                resolved_topic = rospy.resolve_name(button_topic)

            except Exception:
                continue

            self._buttons[resolved_topic] = Button(
                self._config.get_button_config(
                    "ROS-buttons",
                    button_topic)
            )

            self._subscriptions[button_topic] = sub

    def _ros_topic_button_callback(self, data, topic):
        if data.data:
            self._buttons[topic].toggle_state()

    def _ros_button_status_callback(self, data):
        import pdb
        pdb.set_trace()

    def _API_button_callback(self, button_press):
        name = button_press.bitset.bits[0].key
        value = button_press.bitset.bits[0].value

        button_name = ButtonHandler.get_API_button_name(name)

        if button_name not in self._buttons:
            return

        if value:
            self._buttons[button_name].toggle_state()

    @staticmethod
    def get_API_button_name(base_name):
        return f"api.{base_name}"

    @staticmethod
    def get_ROS_button_name(topic):
        return topic


class Button:
    """Encapsulates information for a button"""

    def __init__(self, config: ButtonConfiguration):
        self._state = config.get("initial_state", False)
        self._output_topic = config.get("output_topic")
        self._config = config

        self._pub = rospy.Publisher(
            self._output_topic,
            RosBool,
            queue_size=1)

    def set_state(self, state):
        if self._state == state:
            return

        self._state = state
        self._handle_state_change()

    def toggle_state(self):
        """Toggle the state of the button."""
        self.set_state(not self._state)

    def _handle_state_change(self):
        """Handle the change of state on a ROS button."""
        self._pub.publish(self._state)
