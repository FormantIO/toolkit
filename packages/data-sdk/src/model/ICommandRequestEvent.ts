import { ICommandEventBase } from "./ICommandEventBase";
import { Uuid } from "./Uuid";

// Event generated when a new command is created
export interface ICommandRequestEvent
    extends ICommandEventBase<"command-request"> {
    userId?: Uuid;
}
