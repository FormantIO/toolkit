import { useLayoutEffect, useState } from "react";
import { IAggregateConfiguration } from "../types";

const fetchConfiguration = async () => {
  try {
    const response = await fetch("/config.json");
    const jsonResponse = await response.json();
    return jsonResponse;
  } catch (e) {
    throw e;
  }
};

export const useConfiguration = (): IAggregateConfiguration | null => {
  const [configuration, setConfiguration] =
    useState<IAggregateConfiguration | null>(null);

  useLayoutEffect(() => {
    fetchConfiguration().then((_) => {
      setConfiguration({ ..._ });
    });
  }, []);

  return configuration;
};
