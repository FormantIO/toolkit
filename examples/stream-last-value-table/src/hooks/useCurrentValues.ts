import { useState, useEffect, FC } from "react";
import { useDevice } from "@formant/ui-sdk";
import { updateTable } from "../utils/queryValues";
import { ICurrentValues } from "../types";

export const useCurrentValues = (streams: string[]): ICurrentValues => {
  const device = useDevice();

  const [streamValues, setStreamValues] = useState({});
  useEffect(() => {
    if (!device) return;
    updateTable(device.id, setStreamValues, streams);
  }, [device]);

  return streamValues;
};
