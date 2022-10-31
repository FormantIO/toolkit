import { useEffect, useState } from "react";
import { Authentication, App, KeyValue } from "@formant/data-sdk";
import { baseUrl } from "./config";
import { IAggregateConfiguration } from "./types";

const initialState: IAggregateConfiguration = {
  aggregateType: "sum",
  streamName: "$.host.disk",
  numericSetKey: "utilization",
  aggregateBy: "day",
  numAggregates: 5,
};

const getCloudConfiguration = async () => {
  if (await Authentication.waitTilAuthenticated()) {
    const context = App.getCurrentModuleContext();
    try {
      const result = await fetch(`${baseUrl}/key-value/${context}`, {
        method: "GET",
        headers: {
          "Content-Type": "application/json",
          Authorization: "Bearer " + Authentication.token,
        },
      });
      const parseResult = await result.json();

      if (parseResult.message) return initialState;
      return JSON.parse(parseResult.value);
    } catch (e) {
      console.log(e);
      return initialState;
    }
  }

  return initialState;
};

export const useConfiguration = (
  setter: (_: IAggregateConfiguration) => void
) => {
  const [isLoading, setIsLoading] = useState(true);
  useEffect(() => {
    getCloudConfiguration().then((_) => {
      console.log(_);
      if (!!_) {
        setter(_);
        setIsLoading(false);
      }
    });
  }, []);

  return isLoading;
};
