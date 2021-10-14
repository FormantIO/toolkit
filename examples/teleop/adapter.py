import random
import time
import json

from formant.sdk.agent.v1 import Client as FormantClient


def joystick_callback(message):
    print(str(message))


def string_command_callback(message):
    print(str(message))

if __name__ == "__main__":
    c = FormantClient()
    c.register_custom_data_channel_message_callback(
        joystick_callback, channel_name_filter=["joystick"])
    c.register_custom_data_channel_message_callback(
        string_command_callback, channel_name_filter=["string_command"])

    while True:
        time.sleep(0.1)
