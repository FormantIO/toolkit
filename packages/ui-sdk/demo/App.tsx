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
    <AuthPage
      // backgroundImage={bg}
      cardBackgroundColor="white"
      backgroundColor="white"
      inputSXProps={{
        width: "auto",
        marginBottom: 2,
        marginTop: 2,
        backgroundColor: "red",
        color: "red",
        borderRadius: 10,
      }}
      cardSXProps={{
        padding: "64px 112px",
        borderRadius: 8,
      }}
      logoProps={{ width: "50%", marginBottom: 20 }}
      loginButtonProps={{
        backgroundColor: "#ff8ab7",
        height: 50,
        borderRadius: 8,
        marginTop: 34,
        color: "black",
      }}
      signInWithGoogleButtonProps={{
        height: 30,
        background: "white",
        color: "black",
      }}
    />
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
