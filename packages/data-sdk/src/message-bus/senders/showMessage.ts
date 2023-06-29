import { sendAppMessage } from "./sendAppMessage";

export function showMessage(message: string) {
  sendAppMessage({ type: "show_message", message });
}
