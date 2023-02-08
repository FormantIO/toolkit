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
} from "../src/main";

import { Card } from "../src/components/Charts/LineChart/Card";

import { ServiceParameters } from "../src/components/JsonSchemaForm/ServiceParameters";

function App() {
  const device = useDevice();
  const [params, setParams] = React.useState<ServiceParameters>({});

  const sample = ["device one", "device two"];

  const [en, setEn] = React.useState<string[]>([]);

  return (
    <>
      {device && <RealtimeVideoPlayer deviceId={device?.id} />}
      <BarChart
        height={100}
        width={100}
        labels={["stream a", "stream b"]}
        data={[100, 50]}
      />
      <DoughnutChart
        size={100}
        labels={[
          "donaaaaaaaaaaaaaaaaaaaaaaaaaaae",
          "bbbbbbbbbbbbbbbbbbbbbbbbbbb",
          "cccccccccccccccccccccccccccc",
        ]}
        data={[100, 20, 30]}
      />
      <DoughnutChart
        size={100}
        labels={[
          "donaaaaaaaaaaaaaaaaaaaaaaaaaaae",
          "bbbbbbbbbbbbbbbbbbbbbbbbbbb",
          "cccccccccccccccccccccccccccc",
        ]}
        data={[100, 20, 30]}
      />
      <BubbleChart
        labels={["one", "two", "three"]}
        data={[
          { x: 10, y: 10, r: 2 },
          { x: 15, y: 1, r: 2 },
          { x: 11, y: 5, r: 1 },
        ]}
      />
      <LineChart
        color="#FF0000"
        data={[
          { x: 10, y: 20 },
          { x: 15, y: 25 },
          { x: 20, y: 40 },
        ]}
      />
      <LoadingIndicator />
      <LineChart
        color="#FF0000"
        height={100}
        width={100}
        data={[
          { x: 10, y: 50 },
          { x: 15, y: 50 },
          { x: 20, y: 50 },
          { x: 25, y: 50 },
        ]}
      />
      <PolarChart
        labels={[
          "donaaaaaaaaaaaaaaaaaaaaaaaaaaae",
          "bbbbbbbbbbbbbbbbbbbbbbbbbbb",
          "cccccccccccccccccccccccccccc",
        ]}
        data={[100, 20, 30]}
      />
      <RadarChart
        labels={[
          "donaaaaaaaaaaaaaaaaaaaaaaaaaaae",
          "bbbbbbbbbbbbbbbbbbbbbbbbbbb",
          "cccccccccccccccccccccccccccc",
        ]}
        data={[100, 20, 30]}
      />
      <ScatterChart
        data={[
          { x: 10, y: 20 },
          { x: 15, y: 25 },
          { x: 20, y: 40 },
        ]}
      />
      <ListPicker options={sample} list={en} setList={setEn} />
      <LoadingIndicator />
    </>
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
