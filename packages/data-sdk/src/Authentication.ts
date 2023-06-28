import { FORMANT_API_URL } from "./config";
import { IAuthenticationStore } from "./stores/IAuthenticationStore";
import { AuthenticationStore } from "./stores/AuthenticationStore";
import { getCurrentModuleContext } from "./utils/getCurrentModuleContext";
import { sendAppMessage } from "./message-bus/sendAppMessage";
import { EmbeddedAppMessage } from "./message-bus/EmbeddedAppMessage";

function refreshAuthToken() {
  const moduleName = getCurrentModuleContext();
  if (!moduleName) {
    throw new Error("No module context");
  }
  sendAppMessage({
    type: "refresh_auth_token",
    module: moduleName,
  });
}

function addAccessTokenRefreshListener(
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

export const Authentication: IAuthenticationStore = new AuthenticationStore({
  apiUrl: FORMANT_API_URL,
  refreshAuthToken,
  addAccessTokenRefreshListener: addAccessTokenRefreshListener,
});
