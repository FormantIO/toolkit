import { useState, useEffect, FC, useCallback, useMemo } from "react";
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
import { ServiceParameters } from "./ServiceParameters";
import { getDefaultParams } from "./getDefaultParams";
import { JsonObjectSchema } from "./JsonSchemaForm/types";

type Services = { [key: string]: JsonObjectSchema };

const getServices = async (latestTelemetry: any): Promise<Services> => {
  const newTelp = latestTelemetry.filter(
    (stream: any) => stream.streamName === "ros.service.json"
  );
  const newServices = await fetch(newTelp[0].currentValue);
  const jsonResponse = await fetch(newServices.url);
  return await jsonResponse.json();
};

const App: FC = () => {
  const latestTelemetry = useLatestTelemetry();
  const device = useDevice();
  const [services, setServices] = useState<Services | null>(y);
  const [service, setService] = useState<string | null>(null);
  const [showSnackbar, setShowSnackbar] = useState(false);
  const [params, setParams] = useState<ServiceParameters>({});

  useEffect(() => {
    if (!latestTelemetry) return;
    getServices(latestTelemetry).then((json) => setServices(json));
  }, [latestTelemetry]);

  const handleSubmit = useCallback(() => {
    if (!device || !service) return;
    device.sendCommand(
      "ROS Service Center",
      JSON.stringify({ [service]: params })
    );
    setService(null);
    setShowSnackbar(true);
    setParams({});
  }, [services]);

  const handleSelectService = useCallback(
    (val: string) => {
      if (services) {
        setService(val);
        setParams(getDefaultParams(services[val]));
      }
    },
    [services]
  );

  const dropDownItems = useMemo(
    () =>
      Object.keys(services ?? {}).map((key) => {
        return {
          label: key,
          value: key,
        };
      }),
    [service]
  );

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
          onChange={handleSelectService}
          label="Service"
          value={service ?? ""}
          items={dropDownItems}
        />
        {services && service && (
          <JsonSchemaForm
            schema={services[service]}
            params={params}
            path={[]}
            setParams={setParams}
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
