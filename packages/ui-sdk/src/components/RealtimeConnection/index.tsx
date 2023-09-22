import React, {
  Children,
  FC,
  cloneElement,
  isValidElement,
  useLayoutEffect,
  useState,
} from "react";
import {
  Authentication,
  Device,
  Fleet,
  IStartRealtimeConnectionOptions,
  SessionType,
} from "@formant/data-sdk";

interface IRealtimeConnectionProps {
  children: React.ReactChild[] | React.ReactChild;
  config?: IStartRealtimeConnectionOptions;
  deviceId?: string;
  useCurrentDevice?: boolean;
}

const startConnection = async (
  device: Device,
  config?: IStartRealtimeConnectionOptions
) => {
  await Authentication.waitTilAuthenticated();
  try {
    await device.startRealtimeConnection(config || SessionType.Observe);
  } catch (e) {
    console.error(e);
  }
};

const getDevice = async (deviceId?: string, useCurrentDevice?: boolean) => {
  if (useCurrentDevice === true) return await Fleet.getCurrentDevice();
  if (deviceId && deviceId.length > 0) return await Fleet.getDevice(deviceId);
  return null;
};

const handleRealtimeConnection = async (
  deviceId?: string,
  useCurrentDevice?: boolean,
  config?: IStartRealtimeConnectionOptions
) => {
  const device = await getDevice(deviceId, useCurrentDevice);
  if (device) {
    await startConnection(device, config);
    return device;
  }
  return null;
};

export const RealtimeConnection: FC<IRealtimeConnectionProps> = ({
  children,
  deviceId,
  useCurrentDevice,
  config,
}) => {
  const [loading, setIsLoading] = useState(true);
  const [device, setDevice] = useState<Device>();

  useLayoutEffect(() => {
    if (!deviceId && !useCurrentDevice) return;
    console.warn("start connection");
    handleRealtimeConnection(deviceId, useCurrentDevice, config).then((_) => {
      if (_ === null) {
        console.warn("Unable to create connection ");
        setIsLoading(false);
        return;
      }
      setDevice(_);
      setIsLoading(false);
      console.warn("connection created");
    });
  }, [deviceId, useCurrentDevice]);

  return (
    <>
      {!loading &&
        Children.map(children, (child) => {
          if (isValidElement(child) && device) {
            const oldProps = child.props as any;
            return cloneElement(child, { ...oldProps, device: device } as {
              [key: string]: Device;
            });
          }
          return child;
        })}
    </>
  );
};
