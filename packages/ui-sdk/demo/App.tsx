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
} from "../src/main";

import { Card } from "../src/components/Charts/LineChart/Card";

import { ServiceParameters } from "../src/components/JsonSchemaForm/ServiceParameters";

import { Authentication, Fleet } from "@formant/data-sdk";

function App() {
  const device = useDevice();
  const [params, setParams] = React.useState<ServiceParameters>({});

  const sample = ["device one", "device two"];

  const [en, setEn] = React.useState<string[]>([]);

  React.useEffect(() => {
    if (!device) return;
    device.startRealtimeConnection();
  }, [device]);

  return (
    <div
      style={{
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        minHeight: "100vh",
      }}
    >
      {!device ? (
        <LoadingIndicator />
      ) : (
        <>
          <RealtimeVideoPlayer deviceId={device.id} />
          <Joystick
            joystickConfiguration={{
              position: "left",
              x: {
                dimension: "angular-z",
                scale: 1,
                expo: 2,
                gamepadAxis: 2,
              },
              y: {
                dimension: "linear-x",
                scale: 1,
                expo: 2,
                gamepadAxis: 3,
              },
            }}
            onSendTwistValues={(twistValues, b) => {
              device.sendRealtimeMessage({
                header: {
                  stream: {
                    entityId: device.id,
                    streamName: "/turtle1/cmd_vel",
                    streamType: "twist",
                  },
                  created: 0,
                },
                payload: {
                  twist: {
                    linear: { x: twistValues[1].value, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: twistValues[0].value },
                  },
                },
              });
            }}
            armed
          />
        </>
      )}
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
