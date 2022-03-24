import { IsoDate } from "./IsoDate";
import { ITags } from "./ITags";
import { StreamType } from "./StreamType";
import { Uuid } from "./Uuid";

// Result of a command
// Sent by device when responding to a command
export interface ICommandResponse {
    time: IsoDate;
    message: string;
    success: boolean;
    replyToCommandRequestId: Uuid;
    streamName?: string;
    streamType?: StreamType;
    tags?: ITags;
}
