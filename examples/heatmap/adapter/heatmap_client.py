import os
import datetime
import time
from random import randint


from formant.sdk.agent.v1 import Client as AgentClient


class HeatmapClient:
    def __init__(self) -> None:
        agent_url = "unix:///var/lib/formant/agent.sock"
        self.agent_url = os.getenv("AGENT_URL", agent_url)
        self._geolocation_stream_name = os.getenv(
            "LOCATION_STREAM_NAME", "heatmap_point_location"
        )
        self._numeric_stream_name = os.getenv(
            "NUMERIC_STREAM_NAME", "heatmap_point_weight"
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
                # Around California:
                # https://www.latlong.net/place/san-francisco-ca-usa-594.html
                # https://www.latlong.net/place/los-angeles-ca-usa-1531.html
                latitude = randint(34000000, 38000000)
                longitude = randint(-122000000, -118000000)
                latitude = str(latitude)
                latitude = latitude[0:2] + "." + latitude[2:]
                latitude = float(latitude)
                longitude = str(longitude)
                longitude = longitude[0:4] + "." + longitude[4:]
                longitude = float(longitude)

                weight = randint(1, 50)
                time.sleep(1)
                self._publish_to_heatmap(
                    latitude=latitude, longitude=longitude, weight=weight
                )
        except KeyboardInterrupt:
            pass
