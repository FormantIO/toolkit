import { IOnDemandPresenceTimeRange } from "./IOnDemandPresenceTimeRange";
import { StreamType } from "./StreamType";

export interface IOnDemandPresenceStreamItemGroup {
  datapointType: StreamType;
  timeRanges: IOnDemandPresenceTimeRange[];
}
