import "./App.css";
import { Authentication, Fleet, Device } from "@formant/data-sdk";
import { useLayoutEffect, useState } from "react";
import { DownloadBtn } from "./components/DownloadBtn";

function App() {
  const [device, setDevice] = useState<undefined | Device>();

  useLayoutEffect(() => {
    getCurrentDevice();
  }, []);

  const getCurrentDevice = async () => {
    if (await Authentication.waitTilAuthenticated()) {
      const currentDevice = await Fleet.getCurrentDevice();
      setDevice(currentDevice);
    }
  };
  return (
    <div className="App">
      <DownloadBtn device={device} />
    </div>
  );
}

export default App;
