import time
import json

from formant.sdk.agent.v1 import Client as FormantAgentClient


REQUEST_RESPONSE_CHANNEL_NAME = "request_response_channel"

fclient = FormantAgentClient(ignore_unavailable=True)


# Set up request handler for the given channel name.
@fclient.custom_data_channel_request_handler(REQUEST_RESPONSE_CHANNEL_NAME)
def handler(request_data):
    # Do something with request data (string)
    print(json.loads(request_data))

    # Return response data (string)
    return json.dumps({"message": "Request acknowledged."})


while True:
    time.sleep(1.0)
