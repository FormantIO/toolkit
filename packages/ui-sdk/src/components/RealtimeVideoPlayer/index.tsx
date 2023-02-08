import { Authentication, Device, Fleet, SessionType } from "@formant/data-sdk";
import { RealtimePlayer } from "@formant/ui-sdk-realtime-player";
import "@formant/ui-sdk-realtime-player";
import React, {
  FC,
  useCallback,
  useLayoutEffect,
  useState,
} from "react";
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
  deviceId: string;
  cameraName?: string;
}

export const RealtimeVideoPlayer: FC<IRealtimeVideoPlayerProps> = ({
  deviceId,
  cameraName = "",
}) => {
  const [loading, setIsLoading] = useState(true);

  const start = async (realTimePlayer: RealtimePlayer) => {
    await Authentication.waitTilAuthenticated();
    const device = await Fleet.getDevice(deviceId);
    await device.startRealtimeConnection(SessionType.Observe);
    const videoStream = await getCamera(device);
    device.addRealtimeListener((_, message) => {
      realTimePlayer.drawVideoFrame(message.payload.h264VideoFrame);
    });
    device.startListeningToRealtimeVideo(videoStream);
    setIsLoading(false);
  };
  const stop = async () => {
    const device = await Fleet.getCurrentDevice();
    const videoStream = await getCamera(device);
    await device.stopListeningToRealtimeVideo(videoStream);
  };

  const getCamera = useCallback(
    async (device: Device) => {
      const videoStreams = await device.getRealtimeVideoStreams();
      let videoStream = videoStreams[0];
      if (cameraName)
        videoStream = videoStreams.filter((c) => c.name === cameraName)[0];
      return videoStream;
    },
    [cameraName]
  );

  useLayoutEffect(() => {
    if (!deviceId) return;
    (async () => {
      const realTimePlayer = document.querySelector(
        "formant-realtime-player"
      ) as RealtimePlayer;
      realTimePlayer.drawer.start();
      await start(realTimePlayer);
    })();

    return () => {
      stop();
    };
  }, [deviceId]);

  return (
    <div>
      {loading && <LoadingIndicator></LoadingIndicator>}
      <formant-realtime-player
        style={{
          display: loading ? "none" : "block",
          backgroundColor: "transparent",
        }}
      />
    </div>
  );
};
