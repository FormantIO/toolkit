import { Authentication, Fleet } from "@formant/data-sdk";
import * as React from "react";

interface Stream {
  currentValue: any;
  currentValueTime: string;
  deviceId: string;
  id: string;
  organizationId: string;
  streamName: string;
  streamType: string;
  tags?: any;
}

const useStream = (streamName: string, deviceName: string) => {
  const [stream, setStream] = React.useState();

  React.useEffect(() => {
    getStream();
  }, [stream]);

  const getStream = async () => {
    try {
      if (await Authentication.waitTilAuthenticated()) {
        const _devices = await Fleet.getDevices();
        const _device = _devices.filter((_) => _.name === deviceName)[0];
        const _streams = await _device.getLatestTelemetry();
        const _stream = _streams.filter(
          (_: Stream) => _.streamName === streamName
        );

        setStream(_stream[0].currentValue);
      }
    } catch (err) {
      throw new Error(err as string);
    }
  };

  return stream;
};

export default useStream;
