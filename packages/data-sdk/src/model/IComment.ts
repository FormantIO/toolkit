import { IBaseEvent } from "./IBaseEvent";
import { IsoDate } from "./IsoDate";
import { Uuid } from "./Uuid";

export interface IComment extends IBaseEvent<"comment"> {
    editedAt?: IsoDate;
    userId: Uuid;
}
