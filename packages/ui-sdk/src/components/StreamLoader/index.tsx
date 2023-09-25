import {
  Authentication,
  Fleet,
  IDataPoint,
  IStreamTypeMap,
  ITags,
  StreamType,
} from "@formant/data-sdk";
import React, {
  FC,
  useEffect,
  useState,
  Children,
  isValidElement,
  cloneElement,
} from "react";

interface IStreamLoader {
  streams: string[];
  types: StreamType[];
  children: React.ReactChild;
  deviceIds?: string[];
  useCurrentDevice?: boolean;
}

const SECONDS = 1000;
const MINUTES = 60 * SECONDS;

type StreamData = {
  currentValue: IDataPoint<keyof IStreamTypeMap> | undefined;
  deviceId: string;
  name: string;
  type: keyof IStreamTypeMap;
  tags: ITags;
  points: IDataPoint[];
};

const handleQueryTelemetry = async (
  streams: string[],
  types: StreamType[],
  useCurrentDevice: boolean,
  deviceIds?: string[]
) => {
  await Authentication.waitTilAuthenticated();
  const ids = useCurrentDevice
    ? [(await Fleet.getCurrentDevice()).id]
    : deviceIds;

  const currentStreamValues = await Fleet.queryTelemetry({
    deviceIds: ids,
    start: new Date(Date.now() - MINUTES * 2).toISOString(),
    end: new Date(Date.now()).toISOString(),
    names: streams,
    types: types,
  });
  if (currentStreamValues.length === 0) return [];

  return currentStreamValues.map((_) => ({
    ..._,
    currentValue: _.points.at(-1),
  }));
};

export const StreamLoader: FC<IStreamLoader> = ({
  streams,
  types,
  children,
  deviceIds,
  useCurrentDevice = true,
}) => {
  const [state, setState] = useState<StreamData[]>([]);

  useEffect(() => {
    if (!streams || streams.length === 0) return;
    if (!types || types.length === 0) return;
    console.log("here");

    const dataListener = setInterval(() => {
      handleQueryTelemetry(streams, types, useCurrentDevice, deviceIds).then(
        (_) => {
          //   console.log(_);
          setState(_);
        }
      );
    }, 2000);

    return () => {
      clearInterval(dataListener);
    };
  }, [streams, types, deviceIds, useCurrentDevice]);

  return (
    <>
      {Children.map(children, (child) => {
        if (isValidElement(child)) {
          return cloneElement(child, { ["streams"]: state } as {[key: string]: StreamData[]});
        }
        return child;
      })}
    </>
  );
};
