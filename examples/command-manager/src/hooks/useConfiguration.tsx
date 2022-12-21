import { useEffect, useState } from "react";
import { Authentication, App } from "@formant/data-sdk";
import { useDevice } from "@formant/ui-sdk";
import { IConfiguration } from "../types";
import { useCommands } from "./useCommands";

const getConfiguraion = async (): Promise<IConfiguration | null> => {
  if (await Authentication.waitTilAuthenticated()) {
    const configuration = await App.getCurrentModuleConfiguration();
    if (!!configuration) return JSON.parse(configuration);
    return null;
  }
  return null;
};

export const useConfiguration = (): [IConfiguration | null, boolean] => {
  const device = useDevice();
  const commands = useCommands();
  const [config, setConfig] = useState<IConfiguration | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (!device || !commands) return;
    getConfiguraion().then((_) => {
      if (!_) return;
      //Could run over commands array once to get all values
      _.commands.map((command) => ({
        ...command,
        parameterValue: commands.map((availableCommand) => {
          if (availableCommand.name === command.name) {
            return availableCommand.parameterValue;
          }
        })[0],
      }));
      setConfig(_);
      setLoading(false);
    });
  }, [device, commands]);

  return [config, loading];
};
