import { useEffect, useState } from "react";
import { Device } from "@formant/data-sdk";

const getLatestItems = async (device: Device, streamName: string) => {
  const latestTelemetry = await device.getLatestTelemetry();
  const stream = latestTelemetry.filter(
    (_: { streamName: string }) => _.streamName === streamName
  )[0];
  //TODO: handle error when stream does not exist
  //Or has invalid value
  const csv = stream.currentValue.split(",");

  return csv.map((_: string) => ({ value: _, label: _ }));
};

export const useDropDownItems = (
  device: Device | undefined,
  streamName: string | undefined
) => {
  const [items, setItems] = useState([]);

  useEffect(() => {
    if (!device || !streamName) return;
    getLatestItems(device, streamName).then((_) => setItems(_));
  }, [device, streamName]);
  return items;
};
