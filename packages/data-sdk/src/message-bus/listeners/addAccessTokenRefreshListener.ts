import { EmbeddedAppMessage } from "./EmbeddedAppMessage";

export function addAccessTokenRefreshListener(
  handler: (token: string) => void
): () => void {
  function listener(event: MessageEvent<EmbeddedAppMessage>) {
    const msg = event.data;
    if (msg.type === "auth_token") {
      handler(msg.token);
    }
  }

  window.addEventListener("message", listener);
  return () => {
    window.removeEventListener("message", listener);
  };
}
