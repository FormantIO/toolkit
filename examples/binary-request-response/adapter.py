import time

from formant.sdk.agent.v1 import Client as FormantAgentClient


REQUEST_RESPONSE_CHANNEL_NAME = "request_response_channel"

fclient = FormantAgentClient(ignore_unavailable=True)


# Set up request handler for the given channel name.
@fclient.custom_data_channel_binary_request_handler(REQUEST_RESPONSE_CHANNEL_NAME)
def handler(request_data):
    return request_data  # echo back the request data


while True:
    time.sleep(1.0)
