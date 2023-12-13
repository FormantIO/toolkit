import { useEffect, useState } from "react";
import { Authentication, App, Device, Command } from "@formant/data-sdk";
import { useDevice } from "@formant/ui-sdk";

let rtcDevice: Device;
let connected = false;

const getCommands = async (device: Device) => {
  if (await Authentication.waitTilAuthenticated()) {
    const commands = await device.getAvailableCommands();
    return commands.filter((_: any) => _.enabled);
  }
};

export const useCommands = () => {
  const device = useDevice();
  const [commands, setCommands] = useState<Command[] | null>(null);

  useEffect(() => {
    if (!device) return;
    getCommands(device as any).then((_) => {
      if (!_) return;
      setCommands(_);
    });
  }, [device]);

  return commands;
};

export const initRealtimeDevice = (device: Device | undefined) => {
  if (!device || rtcDevice) {
    return;
  }

  rtcDevice = device;

  Authentication.waitTilAuthenticated().then(() => {
    if (!connected) {
      rtcDevice.on('connect', () => {
        connected = true;
        console.log('Device connected');
      });

      rtcDevice.on('disconnect', () => {
        connected = false;
        console.log('Device disconnected');
        // scheduleReconnect();
      });

      rtcDevice.startRealtimeConnection().catch((error) => {
        console.error('Device connection failed', error);;
      });
    }
  });
}

export const sendButtonState = (streamLabel: string, value: boolean) => {
  if (connected) {
    emitButtonState(rtcDevice, streamLabel, value);
  } else {
    console.error('Warning: not connected to device, button state not sent')
  }
};

function emitButtonState(device: Device, streamLabel: string, value: boolean) {
  device.sendRealtimeMessage({
    header: {
      stream: {
        entityId: device.id,
        label: streamLabel,
        streamName: streamLabel.startsWith("/") ? streamLabel : "Buttons",
        streamType: streamLabel.startsWith("/") ? "boolean" : "bitset",
      },
      created: Date.now(),
    },
    payload: streamLabel.startsWith("/") ? {
      boolean: value,
    } : {
      bitset: {
        bits: [{ key: streamLabel, value }],
      },
    },
  });
}
