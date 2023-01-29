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

import { Card } from "../src/components/Charts/LineChart/Card";
import { dummyData } from "../src/components/Charts/LineChart/dummyData";
import { ServiceParameters } from "../src/components/JsonSchemaForm/ServiceParameters";
import { JsonObjectSchema } from "../src/components/JsonSchemaForm/types";
let y: { [key: string]: JsonObjectSchema } = {
  "/rosout/get_loggers": {
    title: "/rosout/get_loggers",
    type: "object",
    properties: {},
  },
  "/rosout/set_logger_level": {
    title: "/rosout/set_logger_level",
    type: "object",
    properties: {
      logger: { type: "string", title: "logger" },
      level: { type: "string", title: "level" },
    },
  },
  random: {
    title: "/random",
    type: "object",
    properties: {
      header: {
        type: "object",
        properties: {
          "header.seq": {
            type: "integer",
            title: "header.seq",
          },
          "header.stamp": {
            type: "object",
            properties: {
              "header.stamp.secs": {
                type: "integer",
                title: "header.stamp.secs",
              },
              "header.stamp.nsecs": {
                type: "integer",
                title: "header.stamp.nsecs",
              },
            },
            title: "header.stamp",
          },
          "header.frame_id": {
            type: "string",
            title: "header.frame_id",
          },
        },
        title: "header",
      },
      seq: {
        type: "object",
        properties: {
          "seq.secs": {
            type: "integer",
            title: "seq.secs",
          },
          "seq.nsecs": {
            type: "integer",
            title: "seq.nsecs",
          },
        },
        title: "seq",
      },
      twist: {
        type: "object",
        properties: {
          "twist.linear": {
            type: "object",
            properties: {
              "twist.linear.x": {
                type: "number",
                title: "twist.linear.x",
              },
              "twist.linear.y": {
                type: "number",
                title: "twist.linear.y",
              },
              "twist.linear.z": {
                type: "number",
                title: "twist.linear.z",
              },
            },
            title: "twist.linear",
          },
          "twist.angular": {
            type: "object",
            properties: {
              "twist.angular.x": {
                type: "number",
                title: "twist.angular.x",
              },
              "twist.angular.y": {
                type: "number",
                title: "twist.angular.y",
              },
              "twist.angular.z": {
                type: "number",
                title: "twist.angular.z",
              },
            },
            title: "twist.angular",
          },
        },
        title: "twist",
      },
      my_int_array: {
        type: "array",
        items: {
          type: "integer",
        },
        title: "my_int_array",
      },
    },
  },
};
function App() {
  const device = useDevice();
  const [params, setParams] = React.useState<ServiceParameters>({});
  let x = y.random.properties.my_int_array;

  const sample = ["device one", "device two"];

  const [en, setEn] = React.useState<string[]>([]);

  return (
    <>
      {device && <RealtimeVideoPlayer deviceId={device?.id} />}
      <BarChart labels={["stream a", "stream b"]} data={[100, 50]} />
      <LoadingIndicator />
      <button onClick={() => console.log(en)}>Clicl</button>
      <ListPicker options={sample} list={en} setList={setEn} />
      <JsonSchemaForm
        params={params}
        path={[]}
        setParams={setParams}
        schema={x}
      />
    </>
  );
}

const container = document.getElementById("app");
if (container) {
  const root = createRoot(container);
  root.render(
    <FormantProvider>
      <App />
    </FormantProvider>
  );
}
