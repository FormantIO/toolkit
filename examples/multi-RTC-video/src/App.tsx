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
import { useEffect, useMemo } from "react";

function App() {
  const context = useFormant();
  const config = context.configuration as IConfiguration;
  const device = useDevice();

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

  return !device || !context ? (
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
  width: 50%;
`;

export default App;
