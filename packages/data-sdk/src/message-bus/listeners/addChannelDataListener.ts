import { EmbeddedAppMessage } from "./EmbeddedAppMessage";

export function addChannelDataListener(
  channel: string,
  handler: (e: { source: string; data: any }) => void
) {
  const listener = (event: MessageEvent<EmbeddedAppMessage>) => {
    const msg = event.data;
    if (msg.type === "channel_data" && msg.channel === channel) {
      handler({
        source: msg.source,
        data: msg.data,
      });
    }
  };

  window.addEventListener("message", listener);
  return () => window.removeEventListener("message", listener);
}
