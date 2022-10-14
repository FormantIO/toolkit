import { INumericAggregate } from "./INumericAggregate";
import { INumericSetAggregateMap } from "./INumericSetAggregateMap";
import { StreamType } from "./StreamType";

export interface ISupportedStreamAggregateTypeMap {
  "numeric set": INumericSetAggregateMap;
  numeric: INumericAggregate;
}

export type IStreamAggregateTypeMap = Omit<
  {
    [_ in StreamType]: undefined;
  },
  keyof ISupportedStreamAggregateTypeMap
> &
  ISupportedStreamAggregateTypeMap;
