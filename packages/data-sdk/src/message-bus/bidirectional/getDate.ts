import { sendAppMessage } from "../senders/sendAppMessage";
import { EmbeddedAppMessage } from "../listeners/EmbeddedAppMessage";

export async function getDate(time?: Date, minTime?: Date, maxTime?: Date) {
  return new Promise((resolve) => {
    sendAppMessage({
      type: "request_date",
      minTime,
      maxTime,
      time,
    });
    const handler = (event: MessageEvent<EmbeddedAppMessage>) => {
      const msg = event.data;
      if (msg.type === "date_response") {
        window.removeEventListener("message", handler);
        resolve(msg.data);
      }
    };
    window.addEventListener("message", handler);
  });
}
