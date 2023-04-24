import {
  useFormant,
  useDevice,
  RealtimeVideoPlayer,
  LoadingIndicator,
  RealtimeConnection,
} from "@formant/ui-sdk";
import { useCallback, useEffect, useState } from "react";
import "./App.css";

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
      setLoading(false);
      return;
    }
    waitForConnection();

    return () => {
      device.stopRealtimeConnection();
    };
  }, [device, camera]);

  return (
    <div className="App">
      {loading ? (
        <LoadingIndicator />
      ) : (
        <RealtimeConnection device={device}>
          <RealtimeVideoPlayer
            cameraName={camera}
            device={device}
            id="rtc-video"
          />
        </RealtimeConnection>
      )}
    </div>
  );
}

export default App;
