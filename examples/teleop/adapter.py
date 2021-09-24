import random
import time
import json

from formant.sdk.agent.v1 import Client as FormantClient


def callback(message):
    print(str(message))


if __name__ == "__main__":
    c = FormantClient(agent_url="unix:///tmp/agent.sock")
    c.register_custom_data_channel_message_callback(callback)
    def f():
        d = {
            "t": time.time(),
            "X": [random.random() for _ in range(10)],
        }
        payload = json.dumps(d).encode("utf-8")
        c.send_on_custom_data_channel("joystick", payload)

    while True:
        time.sleep(0.1)
        f()
