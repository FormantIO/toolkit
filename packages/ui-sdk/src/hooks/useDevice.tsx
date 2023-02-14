import { Authentication, Fleet, Device } from "@formant/data-sdk";
import * as React from "react";

const useDevice = (deviceId?: string): Device => {
  const [device, setDevice] = React.useState<Device | undefined>();

  React.useEffect(() => {
    getCurrentDevice();
  }, []);

  const getCurrentDevice = async () => {
    try {
      if (await Authentication.waitTilAuthenticated()) {
        if (!!deviceId) {
          const _selectedDevice = await Fleet.getDevice(deviceId);
          setDevice(_selectedDevice);
          return;
        }
        const _current = await Fleet.getCurrentDevice();
        setDevice(_current);
      }
    } catch (err) {
      throw new Error(
        "Authentication failed, please authenticate and try again"
      );
    }
  };

  return device;
};

export default useDevice;
