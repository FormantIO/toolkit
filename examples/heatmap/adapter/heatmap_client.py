import os
import datetime
import time
from random import randint

from formant.sdk.agent.v1 import Client as AgentClient


LOCATION_STREAM_NAME = "heatmap.point.location"
NUMERIC_STREAM_NAME = "heatmap.point.weight"
PUBLISH_FREQUENCY = 30

class HeatmapClient:
    def __init__(self) -> None:
        self._geolocation_stream_name = os.getenv(
            "LOCATION_STREAM_NAME", LOCATION_STREAM_NAME
        )
        self._numeric_stream_name = os.getenv(
            "NUMERIC_STREAM_NAME", NUMERIC_STREAM_NAME
        )
        self._publish_frequency = os.getenv(
            "PUBLISH_FREQUENCY", PUBLISH_FREQUENCY
        )
        self._agent_client = AgentClient(
            agent_url=self.agent_url, ignore_throttled=True
        )

    def _publish_to_heatmap(self, latitude, longitude, weight):
        print(f"{datetime.datetime.now()}\nGeolocation: {latitude}, {longitude}")
        self._agent_client.post_geolocation(
            self._geolocation_stream_name, latitude=latitude, longitude=longitude,
        )
        if weight % 5 == 0:
            print(f"Weight: {weight}")
            self._agent_client.post_numeric(self._numeric_stream_name, weight)

    def run(self):
        try:
            while True:
                # Around Fort Collins, CO, where Walter is:
                # ~ 40.72, -104.77
                latitude = str(randint(40700000, 40800000))
                latitude = latitude[0:2] + "." + latitude[2:]
                latitude = float(latitude)
                longitude = str(randint(-104800000, -104700000))
                longitude = longitude[0:4] + "." + longitude[4:]
                longitude = float(longitude)
                weight = randint(1, 50)
                self._publish_to_heatmap(
                    latitude=latitude, longitude=longitude, weight=weight
                )
                time.sleep(self._publish_frequency)
        except KeyboardInterrupt:
            pass
