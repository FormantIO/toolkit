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

interface IStreamListener {
  streams: string[];
  types: StreamType[];
  children: React.ReactChild | React.ReactChild[];
  deviceIds: string[];
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
  deviceIds: string[]
) => {
  await Authentication.waitTilAuthenticated();

  const currentStreamValues = await Fleet.queryTelemetry({
    deviceIds,
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

export const StreamListener: FC<IStreamListener> = ({
  streams,
  types,
  children,
  deviceIds,
}) => {
  const [state, setState] = useState<StreamData[]>([]);

  useEffect(() => {
    if (!streams || streams.length === 0) return;
    if (!types || types.length === 0) return;

    const dataListener = setInterval(() => {
      handleQueryTelemetry(streams, types, deviceIds).then((_) => {
        setState(_);
      });
    }, 2000);

    return () => {
      clearInterval(dataListener);
    };
  }, [streams, types, deviceIds]);

  return (
    <>
      {Children.map(children, (child) => {
        if (isValidElement(child)) {
          return cloneElement(child, { ["streams"]: state } as {
            [key: string]: StreamData[];
          });
        }
        return child;
      })}
    </>
  );
};
