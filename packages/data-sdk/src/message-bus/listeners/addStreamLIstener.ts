import { StreamType } from "../../model/StreamType";
import { IStreamData } from "../../model/IStreamData";
import { EmbeddedAppMessage } from "./EmbeddedAppMessage";
import { QueryStore } from "../../cache/queryStore";

const queryStore = new QueryStore();

export function addStreamListener(
  streamNames: string[],
  streamTypes: StreamType[],
  handler: (response: IStreamData[] | "too much data" | undefined) => void
): () => void {
  const listener = (event: MessageEvent<EmbeddedAppMessage>) => {
    const msg = event.data;
    if (msg.type === "module_data") {
      const { start, end } = msg.queryRange;
      handler(
        queryStore.moduleQuery(
          {},
          streamNames,
          streamTypes,
          new Date(start),
          new Date(end),
          false
        )
      );
    }
  };

  window.addEventListener("message", listener);
  return () => window.removeEventListener("message", listener);
}
