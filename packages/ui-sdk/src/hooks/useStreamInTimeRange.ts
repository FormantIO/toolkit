import { Authentication, Fleet } from "@formant/data-sdk";
import * as React from "react";

type Point = [number, any];

interface TelemetryStream {
  agentId: string;
  deviceId: string;
  name: string;
  points: Point[];
  tags: any;
  type: string;
}

const useStreamInTimeRange = (
  streamName: string,
  deviceName: string,
  range?: number
) => {
  //Range will be in seconds
  const [stream, setStream] = React.useState<Point[]>();

  React.useEffect(() => {
    getStream();
  }, [stream]);

  const timeout = (ms: number) => {
    return new Promise((resolve) => setTimeout(resolve, ms));
  };

  const getStream = async () => {
    await timeout(1000);
    try {
      if (await Authentication.waitTilAuthenticated()) {
        const _devices = await Fleet.getDevices();
        const _device = _devices.filter((_) => _.name === deviceName)[0];
        const start = getStartISODate(range);
        const end = new Date();
        const _telemetry = await _device.getTelemetry(streamName, start, end);
        setStream(_telemetry[0].points);
      }
    } catch (err) {
      throw new Error(err as string);
    }
  };

  return stream;
};

const getStartISODate = (range: number | undefined) => {
  if (range === undefined) range = 1;
  let minuteInMilli = 60000;
  minuteInMilli = minuteInMilli * range;
  let startDate = Date.now();
  startDate = startDate - minuteInMilli;
  return new Date(startDate);
};

export default useStreamInTimeRange;
