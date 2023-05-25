import "./App.css";
import {
  LoadingIndicator,
  RealtimeConnection,
  RealtimeVideoPlayer,
  useDevice,
  useFormant,
} from "@formant/ui-sdk";
import styled from "@emotion/styled";
import { IConfiguration } from "./types";
import { useEffect, useMemo, useState, useCallback } from "react";

function timeout(ms: number) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function App() {
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const device = useDevice();
  const [loading, setLoading] = useState(true);

  const cameras = useMemo(() => {
    if (!config || config.cameras.length === 0) return <></>;
    return config.cameras.map((_, idx) => (
      <Cell key={idx}>
        <RealtimeVideoPlayer
          id={`camera-${idx}`}
          cameraName={_.name}
          device={device}
        />
      </Cell>
    ));
  }, [config, device]);

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
    if (!device || !context) return;
    waitForConnection();
  }, [device, context]);

  return loading ? (
    <PageLoading>
      <LoadingIndicator />
    </PageLoading>
  ) : (
    <RealtimeConnection device={device}>
      <Container>{cameras}</Container>
    </RealtimeConnection>
  );
}

const PageLoading = styled.div`
  min-height: 100vh;
  min-width: 100vw;
  display: flex;
  align-items: center;
  justify-content: center;
`;

const Container = styled.div`
  display: flex;
  flex-wrap: wrap;
  min-height: 100vh;
  width: 100%;
`;

const Cell = styled.div`
  background: black;
  max-width: 50%;
  max-height: 50vh
`;

export default App;
