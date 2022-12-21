import { useEffect, useState } from "react";
import { useSelector } from "react-redux";
import { Device } from "@formant/data-sdk";
import { AppState } from "../app/store";

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

export const useDropDownItems = (device: Device) => {
  const modalState = useSelector((state: AppState) => state.modal.modalState);

  const [items, setItems] = useState([]);

  useEffect(() => {
    if (!device || modalState.streamName.length === 0) return;
    getLatestItems(device, modalState.streamName).then((_) => setItems(_));
  }, [modalState.streamName, modalState.title]);
  return items;
};
