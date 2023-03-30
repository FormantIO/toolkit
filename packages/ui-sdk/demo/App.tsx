import * as React from "react";
import { createRoot } from "react-dom/client";
import {
  FormantProvider,
  Box,
  Button,
  Container,
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
} from "../src/main";

import { Card } from "../src/components/Charts/LineChart/Card";

import { ServiceParameters } from "../src/components/JsonSchemaForm/ServiceParameters";

import { Authentication, Fleet, SessionType } from "@formant/data-sdk";
import { randomUUID } from "crypto";

function App() {
  const device = useDevice();
  const [params, setParams] = React.useState<ServiceParameters>({});

  const sample = ["device one", "device two"];

  const [en, setEn] = React.useState<string[]>([]);
  const [loading, setIsLoading] = React.useState(true);

  // React.useEffect(() => {
  //   if (!device) return;
  //   device
  //     .startRealtimeConnection(SessionType.Observe)
  //     .then((_) => setIsLoading(false));
  // }, [device]);

  const y = [20, 50, 60, 10, 90, 23, 1, 17];

  const z = "";
  if (!device) return <></>;
  return (
    <div
      style={{
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        minHeight: "100vh",
      }}
    >
      <RealtimeConnection device={device}>
        <RealtimeVideoPlayer
          id={"one"}
          device={device}
          cameraName="rtsp.192.168.131.11.554"
        />
        <RealtimeVideoPlayer
          id={"two"}
          device={device}
          cameraName="rtsp.192.168.131.10.axis-mediamedia.ampcamera2"
        />
        <RealtimeVideoPlayer
          id={"three"}
          device={device}
          cameraName="rtsp.192.168.131.10.axis-mediamedia.ampcamera1"
        />
      </RealtimeConnection>
    </div>
  );
}

const container = document.getElementById("app");
if (container) {
  const root = createRoot(container);
  root.render(
    <FormantProvider parseConfiguration>
      <App />
    </FormantProvider>
  );
}
