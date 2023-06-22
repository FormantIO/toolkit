import { Timestamp } from "./Timestamp";

export interface IOnDemandPresenceTimeRange {
  start: Timestamp;
  end: Timestamp;
  byteSize: number;
  itemCount: number;
}
