from turtle import st
from formant.sdk.agent.v1 import Client as AgentClient
import os
import time
from random import randint


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

    def _publish_to_heatmap(self, latitude, longitud, weight):
        self._agent_client.post_geolocation(
            self._geolocation_stream_name, latitude=latitude, longitude=longitud,
        )
        self._agent_client.post_numeric(self._numeric_stream_name, weight)

    def run(self):
        try:
            while True:
                latitude = randint(-8678933143615723, -8678791522979736)
                latitude = str(latitude)
                start_string = latitude[0:3] + "." + latitude[3:]
                latitude = float(start_string)
                longitud = randint(3614002351236823, 3615648409409885)
                longitud = str(longitud)
                longitud = longitud[0:2] + "." + longitud[2:]
                longitud = float(longitud)
                weight = randint(1, 50)
                time.sleep(1)
                print(latitude, longitud, weight)
                self._publish_to_heatmap(
                    latitude=latitude, longitud=longitud, weight=weight
                )
        except KeyboardInterrupt:
            pass
