import { Authentication, Device, Fleet } from "@formant/data-sdk";
import { useEffect, useState } from "react";

const getCurrentDevice = async () => {
  try {
    if(await Authentication.waitTilAuthenticated()){
        const currentDevice = await Fleet.getCurrentDevice();
        return currentDevice;
    }
  } catch (error) {
    throw error;
  }
};

export const Sample = () => {
  const [device, setDevice] = useState<Device>();

  useEffect(() => {
    getCurrentDevice().then((_) => setDevice(_));
  }, []);

  window.addEventListener("joystick", () => {
    device?.sendRealtimeMessage({
      header: {
        stream: {
          entityId: "",
          streamName: "Joystick",
          streamType: "twist",
        },
        created: 0,
      },
      payload: {
        twist: {
          linear: {
            x: 1,
            y: 0,
            z: 0,
          },
          angular: {
            x: 0,
            y: 0,
            z: 1,
          },
        },
      },
    });
  });

  return null;
};
