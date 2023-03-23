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

  const y = [20, 50, 60, 10, 90, 23, 1, 17];
  const z = "";
  return (
    <div
      style={{
        display: "flex",
        alignItems: "center",
        justifyContent: "center",
        minHeight: "100vh",
      }}
    >
      <BarChart
        xTicksFontSize={8}
        data={[20, 50, 60, 10, 90, 23, 1, 17]}
        // tooltipUnits={"%"}
        labels={y.map(
          (_) =>
            "Driver Front Change And Balance, Rear Change And Balance Passenger Front Change And Balance, Rear Change And Balance"
        )}
      />
      <DoughnutChart
        size={300}
        data={[20, 50, 60, 10, 90, 23, 1, 17]}
        labels={y.map(
          (_) =>
            "Driver Front Change And Balance, Rear Change And Balance Passenger Front Change And Balance, Rear Change And Balance"
        )}
      />
      <Chart
        type="doughnut"
        labels={y.map(
          (_) =>
            "Driver Front Change And Balance, Rear Change And Balance Passenger Front Change And Balance, Rear Change And Balance"
        )}
        data={[20, 50, 60, 10, 90, 23, 1, 17]}
      />
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
