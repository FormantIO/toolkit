import { RealtimeDataStream, RealtimeListener } from "./device.types";

export interface IRealtimeSubscriber {
  addRealtimeListener(listener: RealtimeListener): void;
  removeRealtimeListener(listener: RealtimeListener): void;

  startListeningToRealtimeDataStream(stream: RealtimeDataStream): void;
  stopListeningToRealtimeDataStream(stream: RealtimeDataStream): void;
}
