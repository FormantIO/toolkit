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
import { SessionType, App as FormantApp } from "@formant/data-sdk";

function timeout(ms: number) {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function App() {
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const device = useDevice();
  const [loading, setLoading] = useState(true);

  const cameras = useMemo(() => {
    if (!config || config.cameras.length === 0 || !device) return <></>;
    return config.cameras.map((_, i) => {
      if (!_ || !_.name) {
        FormantApp.showMessage("Configuratio not found");
        return <></>;
      }
      return (
        <Cell key={i}>
          <RealtimeVideoPlayer
            id={`camera-${i}`}
            cameraName={_.name}
            device={device}
          />
        </Cell>
      );
    });
  }, [config, device]);

  const waitForConnection = useCallback(async () => {
    let connected = false;
    while (!connected) {
      connected = await device!.isInRealtimeSession();
      console.warn("Waiting for the main connection to establish.");
      await timeout(100);
    }
    console.warn("Main connection completed");
    await device?.startRealtimeConnection({
      sessionType: SessionType.OBSERVE,
      deadlineMs: 20_000,
      maxConnectRetries: 20,
    });
    setLoading(false);
  }, [device]);

  useEffect(() => {
    if (!device || !context) return;
    waitForConnection();
  }, [device, context]);

  return loading || !device || config.cameras.length === 0 ? (
    <PageLoading>
      <LoadingIndicator />
    </PageLoading>
  ) : (
    <Container>{cameras}</Container>
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
  background-color: black;
`;

const Cell = styled.div`
  background: black;
  max-width: 50%;
  max-height: 50vh;
  height: 50vh;
  width: 50vw;
  display: grid;
  place-items: center;
`;

export default App;
