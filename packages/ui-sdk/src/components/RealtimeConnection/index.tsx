import React, { FC, ReactNode, useLayoutEffect, useState } from "react";
import { Authentication, Device, SessionType } from "@formant/data-sdk";

interface IRealtimeConnectionProps {
  device: Device;
  children: ReactNode;
}

const startConnection = async (device: Device) => {
  await Authentication.waitTilAuthenticated();
  await device.startRealtimeConnection(SessionType.Observe);
};

export const RealtimeConnection: FC<IRealtimeConnectionProps> = ({
  device,
  children,
}) => {
  const [loading, setIsLoading] = useState(true);

  useLayoutEffect(() => {
    if (!device) return;
    console.warn("start connection");
    startConnection(device).then((_) => {
      setIsLoading(false);
      console.warn("connection created");
    });
  }, [device]);

  return <>{!loading && children}</>;
};
