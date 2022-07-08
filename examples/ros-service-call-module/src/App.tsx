import { useState, useEffect, FC, useCallback, useRef } from "react";
import {
  Typography,
  Box,
  Button,
  Select,
  useLatestTelemetry,
  useDevice,
  Snackbar,
} from "@formant/ui-sdk";
import "./App.css";
import { JsonSchemaForm } from "./JsonSchemaForm";

const App: FC = () => {
  const latestTelemetry = useLatestTelemetry();
  const device = useDevice();
  const [services, setServices] = useState<any | undefined>();
  const [service, setService] = useState<string | undefined>();
  const [showSnackbar, setShowSnackbar] = useState(false);

  let serviceParameters = useRef({});

  useEffect(() => {
    getServices();
  }, [latestTelemetry]);

  const getServices = async () => {
    if (latestTelemetry) {
      const newTelp = latestTelemetry.filter(
        (stream: any) => stream.streamName === "ros.services.json"
      );
      const newServices = await fetch(newTelp[0].currentValue);
      const jsonResponse = await fetch(newServices.url);
      const json = await jsonResponse.json();
      setServices(json);
    }
  };

  const handleSubmit = useCallback(() => {
    if (device && service) {
      device.sendCommand(
        "ROS Service Center",
        JSON.stringify({ [service]: serviceParameters.current })
      );
      setService(undefined);
      setShowSnackbar(true);
      serviceParameters.current = {};
    }
  }, []);

  return (
    <div className="App">
      <Box position="relative" textAlign="left" width={350}>
        <Typography sx={{ marginBottom: "8px" }} variant="h2">
          ROS Service Command
        </Typography>
        <Box display="flex" flexDirection={"row"}>
          <Box
            width={"100%"}
            display="flex"
            justifyContent="space-between"
          ></Box>
        </Box>

        <Select
          sx={{ width: 350, textAlign: "left", marginBottom: "16px" }}
          onChange={(val) => {
            serviceParameters.current = {};
            setService(val);
          }}
          label="Service"
          value={service ?? ""}
          items={
            services !== undefined
              ? Object.keys(services).map((_, idx) => ({
                  label: _,
                  value: _,
                }))
              : []
          }
        />
        {service && (
          <JsonSchemaForm
            jsonSchemaObject={services[service]}
            currentStateObject={serviceParameters.current}
          />
        )}
        <Button
          sx={{ position: "absolute", bottom: -50, right: 0 }}
          onClick={handleSubmit}
          disabled={service === undefined || services === undefined}
          size="large"
          variant="contained"
          color="secondary"
        >
          Send command
        </Button>
        <Snackbar
          message="Command sent"
          open={showSnackbar}
          onClose={() => setShowSnackbar(false)}
          autoHideDuration={3000}
        />
      </Box>
    </div>
  );
};

export default App;
