import { AppMessage } from "./AppMessage";

export function sendAppMessage(message: AppMessage) {
  if (!(window && window.parent)) {
    throw new Error("cannot send message to non-existent parent");
  }
  window.parent.postMessage(message, "*");
}
