import time
import json
import random

from formant.sdk.agent.v1 import Client as FormantAgentClient


def example_channel_callback(message):
    print(json.loads(message.payload.decode("utf-8")))


if __name__ == "__main__":
    fclient = FormantAgentClient()

    # Listen to data from the custom web application
    fclient.register_custom_data_channel_message_callback(
        example_channel_callback, channel_name_filter=["example_channel"]
    )

    while True:
        time.sleep(0.1)
        # Send data to custom web application every 100 ms
        fclient.send_on_custom_data_channel(
            "example_channel",
            json.dumps({k: random.randint(1, 10) for k in range(3)}).encode("utf-8"),
        )
