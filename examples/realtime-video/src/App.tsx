import {
  useFormant,
  useDevice,
  RealtimeVideoPlayer,
  LoadingIndicator,
} from "@formant/ui-sdk";
import "./App.css";

function App() {
  const context = useFormant();
  const { camera } = context.configuration as { camera: string };
  const device = useDevice();

  return (
    <div className="App">
      {!device || !camera ? (
        <LoadingIndicator />
      ) : (
        <RealtimeVideoPlayer cameraName={camera} deviceId={device.id} />
      )}
    </div>
  );
}

export default App;
