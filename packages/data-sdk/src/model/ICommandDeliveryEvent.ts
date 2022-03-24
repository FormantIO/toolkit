import { ICommandEventBase } from "./ICommandEventBase";

// Event generated when a command has been delivered to the device
export interface ICommandDeliveryEvent
    extends ICommandEventBase<"command-delivery"> {}
