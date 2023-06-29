import { sendAppMessage } from "../senders/sendAppMessage";
import { EmbeddedAppMessage } from "./EmbeddedAppMessage";
import { getCurrentModuleContext } from "../../utils/getCurrentModuleContext";

export interface ModuleData {
  queryRange: QueryRange;
  time: number;
  streams: { [stream_name: string]: Stream };
}

export interface QueryRange {
  start: number;
  end: number;
}

export interface Stream {
  data: StreamData[];
  loading: boolean;
  tooMuchData: boolean;
  type: string;
}

export interface StreamData {
  points: DataPoint[];
  deviceId: string;
  agentId: string;
  name: string;
  tags: { [key: string]: string };
  type: string;
}

export type DataPoint = [number, any];

export function addModuleDataListener(handler: (data: ModuleData) => void) {
  const moduleName = getCurrentModuleContext();
  if (moduleName) {
    sendAppMessage({ type: "request_module_data", module: moduleName });
  }
  const listener = (event: MessageEvent<EmbeddedAppMessage>) => {
    const msg = event.data;
    if (msg.type === "module_data") {
      handler({
        streams: msg.streams,
        time: msg.time,
        queryRange: msg.queryRange,
      });
    }
  };

  window.addEventListener("message", listener);
  return () => window.removeEventListener("message", listener);
}
