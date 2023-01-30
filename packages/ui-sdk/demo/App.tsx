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
  LineChart,
  BarChart,
  DoughnutChart,
  PolarChart,
  BubbleChart,
  ScatterChart,
  RadarChart,
  InputBase,
  JsonSchemaForm,
  LoadingIndicator,
  ListPicker,
  RealtimeVideoPlayer,
  useDevice,
} from "../src/main";

import { ServiceParameters } from "../src/components/JsonSchemaForm/ServiceParameters";

function App() {
  const device = useDevice();
  const [params, setParams] = React.useState<ServiceParameters>({});

  const sample = ["device one", "device two"];

  const [en, setEn] = React.useState<string[]>([]);

  return (
    <>
      {device && <RealtimeVideoPlayer deviceId={device?.id} />}
      <BarChart labels={["stream a", "stream b"]} data={[100, 50]} />
      <LoadingIndicator />
      <button onClick={() => console.log(en)}>Clicl</button>
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
