#!/usr/bin/env python3

import json
import os
from multiprocessing import Lock
try: 
    from typing import Dict, List
    import rospy
    import rostopic
except:
    import sys
    sys.path.insert(1,"/usr/lib/formant/agent/dist-packages/")
    import rospy
    import rostopic
    from typing import Dict, List
from formant.sdk.agent.v1 import Client as FormantClient

from ros_topic_stats import RosTopicStats


class RosDiagnosticsCollector:
    def __init__(self):
        rospy.init_node("ros_diagnostics_collector")
        self._stream_name = os.getenv("STREAM_NAME", "ros_diagnostics")
        agent_url = os.getenv("AGENT_URL", "unix: // /var/lib/formant/agent.sock")

        self._r = rostopic.ROSTopicHz(-1)
        self._fclient = FormantClient(agent_url=agent_url, ignore_throttled=True)
        self._subscribers = {}  # type: Dict[str,rospy.Subscriber]
        self._topic_stats = []  # type: List[RosTopicStats]
        self._requested_topics = []
        self._updated = True
        self._lock = Lock()
        self._refresh_topics()
        self._lookup_timer = rospy.Timer(rospy.Duration(0.2), self._lookup_and_post)
        self._refresh_timer = rospy.Timer(rospy.Duration(10), self._refresh_topics)
        self._fclient.register_command_request_callback(
            f=self._set_desired_topics, command_filter=["update_topic_list"]
        )
        rospy.spin()

    def _set_requested_topics(self, _requested_topics):
        self._requested_topics = _requested_topics

    def _set_updated(self, _update):
        self._updated = _update

    def _set_desired_topics(self, data):
        filter_topics = json.loads(data.text)
        self._set_requested_topics(filter_topics)
        self._set_updated(False)

    def _refresh_topics(self, event=None):
        self._lock.acquire()
        remaining_topics = list(self._subscribers.keys())
        pubs_out, _ = rostopic.get_topic_list()

        for topic_tuple in pubs_out:
            topic_name = topic_tuple[0]
            if self._updated == True:
                if topic_name in remaining_topics:
                    remaining_topics.remove(topic_name)
                    continue
            topic_type = topic_tuple[1]
            self._topic_stats.append(RosTopicStats(topic_name, topic_type))
            self._subscribers[topic_name] = rospy.Subscriber(
                topic_name,
                rospy.AnyMsg,
                self._r.callback_hz,
                callback_args=topic_name,
            )

        if len(self._requested_topics) != 0:
            new_list = []
            for topic in self._topic_stats:
                if topic.name in self._requested_topics:
                    new_list.append(topic)
            self._topic_stats = new_list
            self._set_updated(True)

        for topic in remaining_topics:
            self._subscribers[topic].unregister()
            del self._subscribers[topic]
            self._topic_stats = [
                stat for stat in self._topic_stats if stat.name != topic
            ]
        rospy.sleep(1)
        self._lock.release()

    def _lookup_and_post(self, event=None):
        self._lock.acquire()
        for topic_stat in self._topic_stats:
            topic_name = topic_stat.name
            stats = self._r.get_hz(topic=topic_name)
            hz = 0
            if stats is not None:
                hz = stats[0]
            topic_stat.set_hz(hz)

        json_string = json.dumps([stat.__dict__ for stat in self._topic_stats])
        self._fclient.post_json(self._stream_name, json_string)
        self._lock.release()
