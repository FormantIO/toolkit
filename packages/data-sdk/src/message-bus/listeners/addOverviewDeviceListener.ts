import { EmbeddedAppMessage, IDevice } from "./EmbeddedAppMessage";
import { sendAppMessage } from "../senders/sendAppMessage";

export function addOverviewDeviceListener(
  handler: (devices: IDevice[]) => void
) {
  sendAppMessage({ type: "request_devices" });
  const listener = (event: MessageEvent<EmbeddedAppMessage>) => {
    const msg = event.data;
    if (msg.type === "overview_devices") {
      handler(msg.data);
    }
  };

  window.addEventListener("message", listener);
  return () => window.removeEventListener("message", listener);
}
