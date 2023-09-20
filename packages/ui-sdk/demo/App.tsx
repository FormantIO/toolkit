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
  StreamLoader,
} from "../src/main";

import { Card } from "../src/components/Charts/LineChart/Card";

import { ServiceParameters } from "../src/components/JsonSchemaForm/ServiceParameters";

import { Authentication, Fleet, SessionType } from "@formant/data-sdk";
import { randomUUID } from "crypto";
import styled from "@emotion/styled";
import { Sample } from "./Sample";

function App() {
  // const device = useDevice();
  // const [params, setParams] = React.useState<ServiceParameters>({});
  const [auth, setAuth] = React.useState(false);

  // const sample = ["device one", "device two"];

  // const [en, setEn] = React.useState<string[]>([]);
  // const [loading, setIsLoading] = React.useState(true);

  React.useEffect(() => {
    Authentication.login("alen+prod+demos@formant.io", "Winter13!").then((_) =>
      setAuth(true)
    );
  }, []);

  // React.useEffect(() => {
  //   if (!device) return;
  //   device.on("disconnect", () => {
  //     console.warn("DISCONNECTED");
  //   });
  // }, [device]);

  // const waitForConnection = React.useCallback(async () => {
  //   let connected = false;
  //   while (!connected) {
  //     connected = await device.isInRealtimeSession();
  //     console.warn("Waiting for the main connection to establish.");
  //   }
  //   console.warn("Main connection completed");

  //   setIsLoading(false);
  // }, [device]);

  // React.useEffect(() => {
  //   if (!device) return;
  //   waitForConnection();
  // }, [device]);

  return (
    <div>
      {!auth ? (
        <></>
      ) : (
        <Container>
          <StreamLoader
            streams={["$.host.cpu"]}
            types={["numeric set"]}
            deviceIds={["8b1b3e3e-1c41-4820-99ca-97657374e2c7", "58d7f6e1-899d-4a8a-8c02-4c805cc8227f"]}
            useCurrentDevice={false}
          >
           <Sample />
          </StreamLoader>
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
