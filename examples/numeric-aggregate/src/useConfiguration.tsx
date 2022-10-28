import { useEffect, useState } from "react";
import { Authentication, App, KeyValue } from "@formant/data-sdk";
import { baseUrl } from "./config";
import { useQuery } from "react-query";
import { IAggregateConfiguration } from "./types";

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

      if (parseResult.message) return null;

      return JSON.parse(parseResult.value);
    } catch (e) {
      console.log(e);
      return null;
    }
  }

  return null;
};

export const useConfiguration = (
  setter: (_: IAggregateConfiguration) => void
) => {
  const [isLoading, setIsLoading] = useState(true);
  useEffect(() => {
    getCloudConfiguration().then((_) => {
      if (!!_) {
        setter(_);
        setIsLoading(false);
      }
    });
  }, []);

  return isLoading;
};
