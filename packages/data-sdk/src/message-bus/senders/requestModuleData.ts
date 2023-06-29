import { sendAppMessage } from "./sendAppMessage";
import { getCurrentModuleContext } from "../../utils/getCurrentModuleContext";

export function requestModuleData() {
  const moduleName = getCurrentModuleContext();
  if (!moduleName) {
    throw new Error("No module context");
  }
  sendAppMessage({
    type: "request_module_data",
    module: moduleName,
  });
}
