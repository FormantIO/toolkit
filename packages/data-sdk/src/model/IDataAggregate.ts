import { IStreamAggregateTypeMap } from "./IStreamAggregateTypeMap";
import { StreamType } from "./StreamType";
import { Timestamp } from "./Timestamp";

export type IDataAggregate<T extends StreamType = StreamType> = [
  Timestamp,
  IStreamAggregateTypeMap[T]
];
