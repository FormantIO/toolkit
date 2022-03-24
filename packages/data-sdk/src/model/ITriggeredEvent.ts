import { IBaseEvent } from "./IBaseEvent";
import { IsoDate } from "./IsoDate";
import { Severity } from "./Severity";
import { Uuid } from "./Uuid";

export interface ITriggeredEvent extends IBaseEvent<"triggered-event"> {
    severity: Severity;
    eventTriggerId: Uuid;
    interval: number;
    intervalStart?: IsoDate;
}
