import { ICommandEventBase } from "./ICommandEventBase";

// Event generated when a command has been delivered to the device
export type ICommandDeliveryEvent = ICommandEventBase<"command-delivery">;
