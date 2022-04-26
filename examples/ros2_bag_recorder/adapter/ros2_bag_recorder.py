import os
import subprocess
import time
import shlex
from typing import Optional


from formant.sdk.agent.v1 import Client as FormantClient

TOPICS_TO_RECORD = ["/turtle1/cmd_vel", "/turtle1/pose"]
BAG_PREFIX = "/home/ubuntu/ros2bags"
ROS2BAG_RECORD_COMMAND_STRING = "ros2 bag record -o "
def _get_bag_record_command():
    command = ROS2BAG_RECORD_COMMAND_STRING
    command += BAG_PREFIX
    command += time.time()
    for topic in TOPICS_TO_RECORD:
        command += " %s" % topic
        return shlex.split(command)

class Ros2BagRecorder:
    def __init__(self):
        agent_url = os.getenv(
            "AGENT_URL", "unix:///var/lib/formant/agent.sock")

        self._fclient = FormantClient(
            agent_url=agent_url, ignore_throttled=True)
        self._recording_process = None # type: Optional[subprocess.Popen[bytes]]

    def run(self):
        self._fclient.register_command_request_callback(
            self._start_recording, command_filter=["start_ros2_bag_recording"])

        self._fclient.register_command_request_callback(
            self._stop_recording, command_filter=["stop_ros2_bag_recording"])

    def stop(self):
        self._stop_recording()

    def _start_recording(self):
        command = _get_bag_record_command()
        if self._recording_process is not None:
            print("Error: Cannot start recording, recording already in progress!")
        else:
            print("Starting recording: %s" % str(command))
            try:
                self._recording_process = subprocess.Popen(command)
            except Exception as e:
                print("Failed to start recording: %s" % str(e))
        
    def _stop_recording(self):
        if self._recording_process is not None:
            print("Stopping recording.")
            try:
                self._recording_process.send_signal(subprocess.signal.SIGINT)
                self._recording_process = None
            except Exception as e:
                print("Failed to stop recording: %s" % str(e))
        else:
            print("Error: Cannot stop recording, no recording in progress!")
