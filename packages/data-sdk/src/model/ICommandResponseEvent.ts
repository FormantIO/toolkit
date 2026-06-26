import { ICommandEventBase } from "./ICommandEventBase";

// Event generated when a device responds with a command result
export type ICommandResponseEvent = ICommandEventBase<"command-response">;
