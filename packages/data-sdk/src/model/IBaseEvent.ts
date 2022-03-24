import { EventType } from "./EventType";
import { IBaseEntity } from "./IBaseEntity";
import { IsoDate } from "./IsoDate";
import { ITags } from "./ITags";
import { StreamType } from "./StreamType";
import { Uuid } from "./Uuid";

export interface IBaseEvent<T extends EventType = EventType>
    extends IBaseEntity {
    type?: T;
    organizationId?: Uuid;
    time: IsoDate;
    endTime?: IsoDate | null;
    message: string;
    viewed?: boolean;
    deviceId?: Uuid;
    streamName?: string;
    streamType?: StreamType;
    tags?: ITags;
    notificationEnabled?: boolean;
}
