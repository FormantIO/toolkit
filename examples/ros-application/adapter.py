import time
import json

from formant.sdk.agent.v1 import Client as FormantAgentClient

REQUEST_RESPONSE_CHANNEL_NAME = "request_response_channel"


def parse_request_message(message):
    request = json.loads(message.payload.decode("utf-8"))
    return request["id"], request["data"]


def wrap_response_message(id, data):
    return json.dumps({"id": id, "data": data}).encode("utf-8")


def callback(message):
    # 1. Parse request message
    id, data = parse_request_message(message)

    # 2. Decode the data sent via the `requestChannel.request` method
    decoded = json.loads(data)
    print(decoded)

    # 3. Wrap response with the ID of the request
    response = wrap_response_message(
        id, json.dumps({"message": "Request acknowledged."})
    )

    # 4. Send the wrapped response
    fclient.send_on_custom_data_channel(REQUEST_RESPONSE_CHANNEL_NAME, response)


if __name__ == "__main__":
    fclient = FormantAgentClient(agent_url="unix:///tmp/agent.sock")

    # Listen to data from the custom web application
    fclient.register_custom_data_channel_message_callback(
        callback, channel_name_filter=[REQUEST_RESPONSE_CHANNEL_NAME]
    )

    while True:
        time.sleep(0.1)
