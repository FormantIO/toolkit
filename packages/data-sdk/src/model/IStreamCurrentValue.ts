import { IsoDate } from "./IsoDate";
import { IStreamTypeMap } from "./IStreamTypeMap";
import { ITags } from "./ITags";
import { StreamType } from "./StreamType";
import { Uuid } from "./Uuid";

export interface IStreamCurrentValue<T extends StreamType = any> {
  deviceId: Uuid;
  streamName: string;
  streamType: StreamType;
  tags: ITags;
  currentValue: IStreamTypeMap[T];
  currentValueTime: IsoDate;
}
