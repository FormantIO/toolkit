import { useState, useEffect, FC } from "react";
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
  const [service, setService] = useState<string | undefined>("");
  const [params, setParams] = useState<{ [key: string]: string }>({});
  const [showSnackbar, setShowSnackbar] = useState(false);

  let serviceParameters = {};

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
      // if(Object.keys(json) === )
      console.log(json);
      setServices(json);
    }
  };

  const handleSubmit = () => {
    console.log(serviceParameters);
    // if (device && service) {
    //   device.sendCommand(
    //     "RosServiceTest",
    //     JSON.stringify({ [service]: serviceParameters })
    //   );
    //   setService(undefined);
    //   setShowSnackbar(true);

    //   setParams({});
    // }
  };
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
            setService(val);
            setParams({});
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
            currentStateObject={serviceParameters}
          />
        )}
        <Button
          sx={{ position: "absolute", bottom: -50, right: 0 }}
          onClick={handleSubmit}
          // disabled={
          //   service === undefined ||
          //   (services[service].length > 0 &&
          //     Object.keys(params).length < services[service].length)
          // }
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
