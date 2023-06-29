import { sendAppMessage } from "./sendAppMessage";

export function goToDevice(deviceId: string) {
  sendAppMessage({
    type: "go_to_device",
    deviceId,
  });
}
