import { Uuid } from "./Uuid";
import { IBaseEvent } from "./IBaseEvent";
import { IsoDate } from "./IsoDate";

export interface IRtcSessionRecord<
  T extends "teleop-session-record" | "port-forwarding-session-record"
> extends IBaseEvent<T> {
  sessionId: Uuid;
  userId: Uuid;
  deviceId: Uuid;
  endTime: IsoDate;
}
