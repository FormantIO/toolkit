import { IBaseEvent } from "./IBaseEvent";
import { Severity } from "./Severity";

export interface ICustomEvent extends IBaseEvent<"custom"> {
    severity?: Severity;
}
