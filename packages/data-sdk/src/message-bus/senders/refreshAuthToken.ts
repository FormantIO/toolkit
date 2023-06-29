import { getCurrentModuleContext } from "../../utils/getCurrentModuleContext";
import { sendAppMessage } from "./sendAppMessage";

export function refreshAuthToken() {
  const moduleName = getCurrentModuleContext();
  if (!moduleName) {
    throw new Error("No module context");
  }
  sendAppMessage({
    type: "refresh_auth_token",
    module: moduleName,
  });
}
