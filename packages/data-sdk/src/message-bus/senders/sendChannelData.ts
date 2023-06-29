import { sendAppMessage } from "./sendAppMessage";
import { getCurrentModuleContext } from "../../utils/getCurrentModuleContext";

export function sendChannelData(channel: string, data: any) {
  const moduleName = getCurrentModuleContext();
  if (!moduleName) {
    throw new Error("No module context");
  }

  sendAppMessage({
    type: "send_channel_data",
    source: moduleName,
    channel,
    data,
  });
}
