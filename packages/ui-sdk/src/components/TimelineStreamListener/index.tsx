import {
  App,
  Authentication,
  Fleet,
  IStreamData,
  IStreamTypeMap,
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

export interface ITimeLineStreamListener {
  streams: string[];
  types: StreamType[];
  children: React.ReactChild | React.ReactChild[];
}

export type TimelineStreamData<T extends StreamType = StreamType> = {
  [key: string]: IStreamTypeMap[T];
};

const handleTimeLine = async (
  streams: string[],
  types: StreamType[],
  setter: (_: TimelineStreamData) => void
) => {
  await Authentication.waitTilAuthenticated();
  const device = await Fleet.getCurrentDevice();

  App.addStreamListener(streams, types, (s) => {
    if (!s) return null;
    const curentDeviceStream = (
      s as IStreamData<keyof IStreamTypeMap>[]
    ).filter((_) => _.deviceId === device.id);

    const latestData = curentDeviceStream.reduce<TimelineStreamData>((p, c) => {
      const latestPoint = c.points.at(-1);
      if (latestPoint) {
        p[c.name] = latestPoint[1];
        return p;
      }
      return p;
    }, {});

    if (Object.keys(latestData).length > 0) {
      setter(latestData);
    }
    return null;
  });
};

export const TimelineStreamListener: FC<ITimeLineStreamListener> = ({
  streams,
  types,
  children,
}) => {
  const [state, setState] = useState<TimelineStreamData>();

  useEffect(() => {
    if (!streams || streams.length === 0) return;
    if (!types || types.length === 0) return;

    handleTimeLine(streams, types, setState);
  }, [streams, types]);

  return (
    <>
      {Children.map(children, (child) => {
        if (isValidElement(child)) {
          return cloneElement(child, { ["streams"]: state } as {
            [key: string]: TimelineStreamData;
          });
        }
        return child;
      })}
    </>
  );
};
