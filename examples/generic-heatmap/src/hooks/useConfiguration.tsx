import { useEffect, useState } from "react";
import { Authentication, App } from "@formant/data-sdk";
import { useDevice } from "@formant/ui-sdk";
import { IConfiguration } from "../types";

const getConfiguraion = async () => {
  if (await Authentication.waitTilAuthenticated()) {
    const configuration = await App.getCurrentModuleConfiguration();
    if (!!configuration) return JSON.parse(configuration);
    return null;
  }
};

export const useConfiguration = () => {
  const device = useDevice();
  const [config, setConfig] = useState<IConfiguration | null>(null);

  useEffect(() => {
    if (!device) return;
    getConfiguraion().then((_) => {
      setConfig(_);
    });
  }, [device]);

  return config;
};
// locationStream: "heatmap_point_location",
// numericStream: "",
// start: {
//   type: "timeRange",
//   value: "1",
// },
// end: {
//   type: "scrubber",
//   value: "",
// },
