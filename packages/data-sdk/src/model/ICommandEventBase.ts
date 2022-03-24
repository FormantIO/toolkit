import { IBaseEvent } from "./IBaseEvent";
import { Uuid } from "./Uuid";

export interface ICommandEventBase<
    T extends "command-request" | "command-delivery" | "command-response"
> extends IBaseEvent<T> {
    commandId: Uuid;
}
