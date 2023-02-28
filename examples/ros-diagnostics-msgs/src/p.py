from formant.sdk.agent.v1 import Client
import time
import json

c = Client(ignore_throttled=True)
while True:
    time.sleep(2)
    x = {
        "header": {
            "frame_id": "",
            "seq": 875,
            "stamp": {"secs": 1659048794, "nsecs": 999167919},
        },
        "status": [
            {
                "hardware_id": "ABC123",
                "level": 0,
                "message": "Normal operation",
                "name": "Arm component",
                "values": [
                    {
                        "key": "Angle",
                        "value": "45 degrees"
                    },
                    {
                        "key": "Speed",
                        "value": "10 RPM"
                    }
                ],
            },
        ],
    }
    c.post_json("Diagnostics", json.dumps(x))
