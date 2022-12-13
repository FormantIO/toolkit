import { useEffect, useState } from "react";
import { Authentication, App } from "@formant/data-sdk";
import { useDevice } from "@formant/ui-sdk";
import { IAggregateConfiguration } from "../types";

const getConfiguraion = async () => {
  if (await Authentication.waitTilAuthenticated()) {
    const configuration = await App.getCurrentModuleConfiguration();
    if (!!configuration) return JSON.parse(configuration);
    return null;
  }
};

export const useConfiguration = (): [
  IAggregateConfiguration | null,
  boolean
] => {
  const device = useDevice();
  const [config, setConfig] = useState<IAggregateConfiguration | null>(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    if (!device) return;
    getConfiguraion().then((_) => {
      Object.keys(_).includes("deviceIds")
        ? setConfig(_)
        : setConfig({ ..._, deviceIds: [device.id] });
      setLoading(false);
    });
  }, [device]);

  return [config, loading];
};
