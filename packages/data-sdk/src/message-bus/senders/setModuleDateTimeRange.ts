import { sendAppMessage } from "./sendAppMessage";
import { getCurrentModuleContext } from "../../utils/getCurrentModuleContext";

export function setModuleDateTimeRange(
  beforeInMilliseconds: number,
  afterInMilliseconds?: number
) {
  const moduleName = getCurrentModuleContext();
  if (!moduleName) {
    throw new Error("No module context");
  }
  sendAppMessage({
    type: "set_module_data_time_range",
    module: moduleName,
    before: beforeInMilliseconds,
    after: afterInMilliseconds || 0,
  });
}
