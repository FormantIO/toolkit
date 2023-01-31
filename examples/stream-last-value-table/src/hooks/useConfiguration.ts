import { useLayoutEffect, useState } from "react";
import { lastKnowValue, Iconfiguration } from "../types";
import { getConfiguration } from "../utils/getConfiguration";

const fetchConfiguration = async () => {
  try {
    const response = await fetch("/config.json");
    const jsonResponse = await response.json();

    return jsonResponse;
  } catch (e) {
    throw e;
  }
};

export const useConfiguration = () => {
  const [configuration, setConfiguration] = useState<{
    streams: Iconfiguration[];
  }>({
    streams: [],
  });

  useLayoutEffect(() => {
    fetchConfiguration().then((_) => {
      setConfiguration(_);
    });
  }, []);

  return configuration;
};
