import {
  useFormant,
  useDevice,
  RealtimeVideoPlayer,
  LoadingIndicator,
  RealtimeConnection,
} from "@formant/ui-sdk";
import { useCallback, useEffect, useState } from "react";
import "./App.css";
import { SessionType } from "@formant/data-sdk";
import { RosConnection, Subscriber, useMsg } from "rosreact";
import { Ros, Topic } from "roslib";

function timeout(ms: number) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function App() {
  const context = useFormant();
  //context.configuration
  const { localTwistTopic, formantTwistStream } = {
    localTwistTopic: "/cmd_vel_stamped",
    formantTwistStream: "/cmd_vel_stamped",
  } as {
    localTwistTopic: string;
    formantTwistStream: string;
  };

  const device = useDevice();
  const [loading, setLoading] = useState(true);
  const [data, setData] = useState(undefined);

  useEffect(() => {
    if (loading === false && device) {
      const ros = new Ros({
        url: "ws://localhost:9090",
      });

      ros.on("connection", function () {
        console.log("Connected to websocket server.");
      });

      ros.on("error", function (error) {
        console.log("Error connecting to websocket server: ", error);
      });

      ros.on("close", function () {
        console.log("Connection to websocket server closed.");
      });

      const listener = new Topic({
        ros: ros,
        name: localTwistTopic,
        messageType: "geometry_msgs/TwistStamped",
      });
      listener.subscribe(function (message: any) {
        console.log(
          "Received message on " +
            listener.name +
            ": " +
            JSON.stringify(message)
        );
        setData(message);
        device.sendRealtimeMessage({
          header: {
            stream: {
              entityId: device.id,
              streamName: formantTwistStream,
              streamType: "twist",
            },
            created:
              message.header.stamp.secs * 1000 + message.header.stamp.nsecs,
          },
          payload: {
            twist: message.twist,
          },
        });
      });
    }
  }, [loading, device]);
  console.log(localTwistTopic);

  const waitForConnection = useCallback(async () => {
    if (!device) return;
    await device.startRealtimeConnection({
      sessionType: SessionType.OBSERVE,
      maxConnectRetries: 10,
      deadlineMs: 10000,
    });
    let connected = false;
    while (!connected) {
      connected = await device.isInRealtimeSession();
      console.warn("Waiting for the main connection to establish.");
      await timeout(2000);
    }
    console.warn("Main connection completed");

    setLoading(false);
  }, [device]);

  useEffect(() => {
    if (!device) return;
    waitForConnection();

    return () => {
      device.stopRealtimeConnection();
    };
  }, [device]);

  return (
    <div className="App">
      {loading || !device ? (
        <LoadingIndicator />
      ) : (
        <p> {`Sending:  ${JSON.stringify(data.twist)}`} </p>
      )}
    </div>
  );
}

export default App;
