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

function timeout(ms: number) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function App() {
  const context = useFormant();
  const { camera, isTeleopModule } = context.configuration as {
    camera: string;
    isTeleopModule: boolean;
  };
  const device = useDevice();
  const [loading, setLoading] = useState(true);

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
    if (!device || !camera) return;
    if (!isTeleopModule) {
      device
        .startRealtimeConnection({
          sessionType: SessionType.OBSERVE,
          maxConnectRetries: 50,
          deadlineMs: 15000,
        })
        .then(() => setLoading(false));
      return;
    }
    waitForConnection();

    return () => {
      device.stopRealtimeConnection();
    };
  }, [device, camera]);

  return (
    <div className="App">
      {loading || !device ? (
        <LoadingIndicator />
      ) : (
        <RealtimeVideoPlayer
          cameraName={camera}
          device={device!}
          id="rtc-video"
        />
      )}
    </div>
  );
}

export default App;
