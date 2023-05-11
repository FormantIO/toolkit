import * as React from "react";
import { createRoot } from "react-dom/client";
import {
  FormantProvider,
  Box,
  Button,
  Grid,
  Typography,
  Select,
  TextField,
  Tooltip,
  HelpIcon,
  Dialog,
  DialogTitle,
  DialogContent,
  DialogContentText,
  DialogActions,
  Switch,
  Stack,
  Link,
  Snackbar,
  icons,
  Icon,
  InputBase,
  JsonSchemaForm,
  BarChart,
  LoadingIndicator,
  RealtimeVideoPlayer,
  useDevice,
  ListPicker,
  DoughnutChart,
  LineChart,
  BubbleChart,
  PolarChart,
  RadarChart,
  ScatterChart,
  Joystick,
  Chart,
  RealtimeConnection,
  AuthPage,
} from "../src/main";

import { Card } from "../src/components/Charts/LineChart/Card";

import { ServiceParameters } from "../src/components/JsonSchemaForm/ServiceParameters";

import { Authentication, Fleet, SessionType } from "@formant/data-sdk";
import { randomUUID } from "crypto";
import styled from "@emotion/styled";

function App() {
  const device = useDevice();
  const [params, setParams] = React.useState<ServiceParameters>({});
  const [auth, setAuth] = React.useState(false);

  const sample = ["device one", "device two"];

  const [en, setEn] = React.useState<string[]>([]);
  const [loading, setIsLoading] = React.useState(true);

  // React.useEffect(() => {
  //   Authentication.login("alen+sanity@formant.io", "Winter13!").then((_) =>
  //     setAuth(true)
  //   );
  // }, []);

  React.useEffect(() => {
    if (!device) return;
    device.on("disconnect", () => {
      console.warn("DISCONNECTED");
    });
  }, [device]);

  const waitForConnection = React.useCallback(async () => {
    let connected = false;
    while (!connected) {
      connected = await device.isInRealtimeSession();
      console.warn("Waiting for the main connection to establish.");
    }
    console.warn("Main connection completed");

    setIsLoading(false);
  }, [device]);

  React.useEffect(() => {
    if (!device) return;
    waitForConnection();
  }, [device]);

  return (
    <div>
      {!device || loading ? (
        <></>
      ) : (
        <Container>
          <RealtimeConnection device={device}>
            <Cell>
              <RealtimeVideoPlayer
                device={device}
                id="aa"
                cameraName=""
              />
            </Cell>
            {/* <Cell>
              <RealtimeVideoPlayer
                device={device}
                id="dos"
                cameraName="rtsp.192.168.131.10.axis-mediamedia.ampcamera1"
              />
            </Cell>
            <Cell>
              <RealtimeVideoPlayer
                device={device}
                id="tres"
                cameraName="rtsp.192.168.131.10.axis-mediamedia.ampcamera2"
              />
            </Cell>
            <Cell>
              <RealtimeVideoPlayer
                device={device}
                id="cuatro"
                cameraName="rtsp.192.168.131.11.554"
              />
            </Cell> */}
          </RealtimeConnection>
        </Container>
      )}
    </div>
  );
}

const Container = styled.div`
  height: 100vh;
  width: 100%;
  display: flex;
  flex-wrap: wrap;
  overflow: hidden;
  background-color: black;
`;

const Cell = styled.div`
  height: 50vh;
  width: 50vw;
`;

const container = document.getElementById("app");
if (container) {
  const root = createRoot(container);
  root.render(
    <FormantProvider parseConfiguration>
      <App />
    </FormantProvider>
  );
}
