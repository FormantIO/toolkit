import { StreamType } from "./StreamType";
import { AnalyticsAggregateType } from "./AnalyticsAggregateType";

export interface IStreamColumn {
  streamName: string;
  streamType: StreamType;
  aggregateType?: AnalyticsAggregateType;
}
