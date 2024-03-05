import { EventType } from "./EventType";
import { IBaseEvent } from "./IBaseEvent";
import { IsoDate } from "./IsoDate";
import { Severity } from "./Severity";
import { Uuid } from "./Uuid";

export interface ITriggeredEvent<T extends EventType = EventType>
  extends IBaseEvent<T> {
  severity: Severity;
  eventTriggerId: Uuid;
  interval?: number;
  intervalStart?: IsoDate;
}
