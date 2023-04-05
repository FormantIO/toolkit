import {
  useFormant,
  useDevice,
  RealtimeVideoPlayer,
  LoadingIndicator,
  RealtimeConnection,
} from "@formant/ui-sdk";
import { useEffect } from "react";
import "./App.css";

function App() {
  const context = useFormant();
  const { camera } = context.configuration as { camera: string };
  const device = useDevice();

  return (
    <div className="App">
      {!device || camera === undefined ? (
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
