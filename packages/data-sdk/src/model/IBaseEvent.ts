import { EventType } from "./EventType";
import { IDictionary } from "./IDictionary";
import { IsoDate } from "./IsoDate";
import { ITaggedEntity } from "./ITaggedEntity";
import { StreamType } from "./StreamType";
import { Uuid } from "./Uuid";

export interface IBaseEvent<T extends EventType = EventType>
  extends ITaggedEntity {
  type: T;
  organizationId?: Uuid;
  time: IsoDate;
  endTime?: IsoDate | null;
  parentId?: Uuid | null;
  metadata?: IDictionary;
  message: string;
  viewed?: boolean;
  deviceId?: Uuid;
  streamName?: string;
  streamType?: StreamType;
  notificationEnabled?: boolean;
  eventTriggerId?: Uuid | null;
  setsDeviceColor?: boolean;
}
