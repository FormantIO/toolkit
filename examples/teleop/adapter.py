import random
import time
import json

from formant.sdk.agent.v1 import Client as FormantClient


def callback(message):
    print(str(message))


if __name__ == "__main__":
    c = FormantClient(agent_url="unix:///tmp/agent.sock")
    c.register_custom_data_channel_message_callback(callback)
    
    while True:
        time.sleep(0.1)
