import { IDataAggregate } from "./IDataAggregate";
import { ITags } from "./ITags";
import { StreamType } from "./StreamType";
import { Uuid } from "./Uuid";

export interface IStreamAggregateData<T extends StreamType = StreamType> {
  deviceId: Uuid;
  name: string;
  type: T;
  tags: ITags;
  aggregates: IDataAggregate<T>[];
}
