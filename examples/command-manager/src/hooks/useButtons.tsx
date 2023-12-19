import { Authentication, Device, SessionType } from "@formant/data-sdk";

let rtcDevice: Device;
let connected = false;
let setConnectedFn: React.Dispatch<React.SetStateAction<boolean>>;
let reconnectTimeout: number | null = null;

export const updateConnectedSetFn = (setConnected: React.Dispatch<React.SetStateAction<boolean>>) => {
  setConnectedFn = setConnected;
}

function setConnected(connectState: boolean) {
  connected = connectState;
  setConnectedFn(connected);
}

export const initRealtimeDevice = (device: Device | undefined) => {
  if (!device || rtcDevice) {
    return;
  }

  rtcDevice = device;

  Authentication.waitTilAuthenticated().then(() => {
    if (!connected) {
      rtcDevice.on('connect', () => {
        setConnected(true);
        console.log('Device connected');
      });

      rtcDevice.on('disconnect', () => {
        setConnected(false);
        console.log('Device disconnected');

        if (reconnectTimeout) {
          clearTimeout(reconnectTimeout);
          reconnectTimeout = setTimeout(() => {
            rtcDevice.startRealtimeConnection(SessionType.HEADLESS).catch((error) => {
              console.error('Device connection failed', error);;
            });
            reconnectTimeout = null;
          }, 3000);
        }
      });

      rtcDevice.startRealtimeConnection(SessionType.HEADLESS).catch((error) => {
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
