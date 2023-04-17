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
  AuthPage,
} from "../src/main";
import bg from "../src/images/bg.jpg";

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

  return (
    <div>
      {!device ? (
        <></>
      ) : (
        <RealtimeConnection device={device}>
          <RealtimeVideoPlayer
            device={device}
            id="aa"
            cameraName="/camera/front/left/image_raw/compressed"
          />
        </RealtimeConnection>
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
