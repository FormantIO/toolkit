import { sendAppMessage } from "./sendAppMessage";
import { getCurrentModuleContext } from "../../utils/getCurrentModuleContext";

export function setupModuleMenus(menus: { label: string }[]) {
  const moduleName = getCurrentModuleContext();
  if (!moduleName) {
    throw new Error("No module context");
  }
  sendAppMessage({
    type: "setup_module_menus",
    module: moduleName,
    menus,
  });
}
