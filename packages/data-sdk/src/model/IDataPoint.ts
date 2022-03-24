import { IStreamTypeMap } from "./IStreamTypeMap";
import { StreamType } from "./StreamType";
import { Timestamp } from "./Timestamp";

export type IDataPoint<T extends StreamType = StreamType> = [
  Timestamp,
  IStreamTypeMap[T]
];
