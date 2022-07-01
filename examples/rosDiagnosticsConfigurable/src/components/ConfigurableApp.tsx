import { useState, FC, useEffect } from "react";
import { ModuleData, App } from "@formant/data-sdk";

export const ConfigurableApp = () => {
  const [latestStats, setLatestStats] = useState<any>({ items: [] });
  const [errorMessage, setErrorMessage] = useState("Waiting for data...");

  useEffect(() => {
    App.addModuleDataListener(receiveModuleData);
  }, []);

  const receiveModuleData = async (newValue: ModuleData) => {
    try {
      const url = getLatestJsonUrl(newValue);
      if (!url) {
        return;
      }
      const items = await (await fetch(url)).json();

      if (latestStats.items.length > 0) {
        if (JSON.stringify(items) !== JSON.stringify(latestStats.items)) {
          setLatestStats({ items });
          return;
        }
        return;
      }

      setLatestStats({ items });
    } catch (error) {
      setLatestStats({ items: [] });
      setErrorMessage(error as string);
    }
  };
};

function getLatestJsonUrl(moduleData: ModuleData): string | undefined {
  const streams = Object.values(moduleData.streams);
  if (streams.length === 0) {
    throw new Error("No streams.");
  }
  const stream = streams[0];
  if (stream === undefined) {
    throw new Error("No stream.");
  }
  if (stream.loading) {
    return undefined;
  }
  if (stream.tooMuchData) {
    throw new Error("Too much data.");
  }
  if (stream.data.length === 0) {
    throw new Error("No data.");
  }
  const latestPoint = stream.data[0].points.at(-1);
  if (!latestPoint) {
    throw new Error("No datapoints.");
  }
  return latestPoint[1];
}
