import { ICommandParameter } from "./ICommandParameter";
import { IsoDate } from "./IsoDate";
import { Uuid } from "./Uuid";

// A command sent to a device when polling for new commands
export interface ICommandRequest {
    id: Uuid; // command ID
    command: string;
    parameter: ICommandParameter;
    createdAt: IsoDate;
}
