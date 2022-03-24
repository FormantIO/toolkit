import { ICommandEventBase } from "./ICommandEventBase";

// Event generated when a device responds with a command result
export interface ICommandResponseEvent
    extends ICommandEventBase<"command-response"> {}
