import time

from formant.sdk.agent.v1 import Client as FormantAgentClient
from protos import example_pb2


def echo(message):
    # message.payload is the protobuf bytes
    # decode
    example = example_pb2.Person.FromString(message.payload)
    print(example)
    # echo it back on the same channel
    fclient.send_on_custom_data_channel("example_channel", example.SerializeToString())


if __name__ == "__main__":
    fclient = FormantAgentClient()

    # Listen to data from the custom web application
    fclient.register_custom_data_channel_message_callback(
        echo, channel_name_filter=["example_channel"]
    )

    while True:
        time.sleep(1.0)
