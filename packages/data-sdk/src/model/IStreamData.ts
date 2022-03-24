import { IDataPoint } from "./IDataPoint";
import { ITags } from "./ITags";
import { StreamType } from "./StreamType";
import { Uuid } from "./Uuid";

export interface IStreamData<T extends StreamType = StreamType> {
  deviceId: Uuid;
  name: string;
  type: T;
  tags: ITags;
  points: IDataPoint<T>[];
}
