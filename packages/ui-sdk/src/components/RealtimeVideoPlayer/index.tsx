import { Authentication, Device, Fleet } from "@formant/data-sdk";
import { RealtimePlayer } from "@formant/ui-sdk-realtime-player";
import "@formant/ui-sdk-realtime-player";
import React, { FC, useCallback, useLayoutEffect, useState } from "react";
import { LoadingIndicator } from "../LoadingIndicator";

declare global {
  namespace JSX {
    interface IntrinsicElements {
      "formant-realtime-player": React.DetailedHTMLProps<
        React.HTMLAttributes<HTMLElement>,
        HTMLElement
      >;
    }
  }
}

interface IRealtimeVideoPlayerProps {
  device: Device;
  id: string;
  cameraName: string;
}

export const RealtimeVideoPlayer: FC<IRealtimeVideoPlayerProps> = ({
  device,
  id,
  cameraName,
}) => {
  const [loading, setIsLoading] = useState(true);

  const start = async (realTimePlayer: RealtimePlayer) => {
    await Authentication.waitTilAuthenticated();
    const videoStream = await getCamera();
    device.addRealtimeListener((_, message) => {
      if (message.header.stream.streamName === cameraName) {
        realTimePlayer.drawVideoFrame(message.payload.h264VideoFrame);
      }
    });
    device.startListeningToRealtimeVideo(videoStream);
    setIsLoading(false);
  };
  const stop = async () => {
    const device = await Fleet.getCurrentDevice();
    const videoStream = await getCamera();
    await device.stopListeningToRealtimeVideo(videoStream);
  };

  const getCamera = useCallback(async () => {
    const videoStreams = await device.getRealtimeVideoStreams();

    const cameras = videoStreams.filter((c) => c.name === cameraName);
    if (cameras.length === 0)
      console.error("Error: Camera name is incorrect or doesn't exist");
    return cameras[0];
  }, [cameraName]);

  const handleMount = useCallback(async () => {
    const realTimePlayer = document.querySelector(`#${id}`) as RealtimePlayer;
    realTimePlayer.drawer.start();
    await start(realTimePlayer);
  }, []);

  useLayoutEffect(() => {
    if (!device) return;
    handleMount().then((_) => console.warn("video mounted"));
    return () => {
      stop();
    };
  }, [device]);

  return (
    <div>
      {loading && <LoadingIndicator></LoadingIndicator>}
      <formant-realtime-player
        id={id}
        style={{
          display: loading ? "none" : "block",
          backgroundColor: "transparent",
        }}
      />
    </div>
  );
};
