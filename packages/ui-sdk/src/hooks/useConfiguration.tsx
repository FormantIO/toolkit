import { App } from "@formant/data-sdk";
import * as React from "react";
import { authenticate } from "../utils/authenticate";

const handleParse = (_: string, parse?: boolean) =>
  !!parse ? JSON.parse(_) : _;

const getCurrentConfiguration = async (setter: any, parse?: boolean) => {
  await authenticate(async () => {
    try {
      const currentConfig = await App.getCurrentModuleConfiguration();
      if (!currentConfig) return;
      const config = handleParse(currentConfig, parse);
      setter(config);

      App.addModuleConfigurationListener((config) => {
        const configuration = handleParse(config.configuration, parse);
        setter(configuration);
      });
    } catch (e) {
      throw e;
    }
  });
};

export const useConfiguration = (config?: { parse: boolean }) => {
  const [configuration, setConfiguration] = React.useState<any>("");

  React.useEffect(() => {
    getCurrentConfiguration(setConfiguration, config?.parse);
  }, []);

  return configuration;
};

export default useConfiguration;
