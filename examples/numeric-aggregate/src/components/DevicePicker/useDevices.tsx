import { Authentication, Fleet, Device } from "@formant/data-sdk";
import { useState, useEffect } from "react";

const queryDevices = async (): Promise<Device[] | undefined> => {
  try {
    if (await Authentication.waitTilAuthenticated()) {
      const devices = await Fleet.getDevices();
      return devices;
    }
  } catch (error) {
    throw error;
  }
};
export const useDevices = () => {
  const [devices, setDevices] = useState<Device[]>([]);

  useEffect(() => {
    queryDevices().then((_) => {
      setDevices(_!);
    });
  }, []);

  return devices;
};
