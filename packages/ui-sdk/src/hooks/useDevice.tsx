import { Authentication, Fleet, Device } from "@formant/data-sdk";
import { useState, useEffect } from "react";

async function getCurrentDevice(deviceId: string | undefined): Promise<Device> {
  if (!(await Authentication.waitTilAuthenticated())) {
    throw new Error("Unauthenticated");
  }

  return !!deviceId
    ? await Fleet.getDevice(deviceId)
    : await Fleet.getCurrentDevice();
}

const useDevice = (deviceId?: string): Device | undefined => {
  const [device, setDevice] = useState<Device | undefined>();

  useEffect(() => {
    getCurrentDevice(deviceId)
      .then((device) => setDevice(device))
      .catch((err: unknown) => {
        console.log("Failed fetching device", { err });
        throw new Error(
          "Authentication failed, please authenticate and try again"
        );
      });
  }, [deviceId]);

  return device;
};

export default useDevice;
