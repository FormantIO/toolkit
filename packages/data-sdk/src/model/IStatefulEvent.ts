import { IBaseEvent } from "./IBaseEvent";
import { Severity } from "./Severity";

export interface IStatefulEvent extends IBaseEvent<"stateful"> {
  severity?: Severity;
  // TODO: Add entrance and exit condition here
}
