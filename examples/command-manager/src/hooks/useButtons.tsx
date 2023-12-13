import { Authentication, Device } from "@formant/data-sdk";

let rtcDevice: Device;
let connected = false;

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
