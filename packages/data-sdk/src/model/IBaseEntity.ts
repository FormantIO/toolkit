import { IsoDate } from "./IsoDate";
import { Uuid } from "./Uuid";

export interface IBaseEntity {
    id?: Uuid;
    createdAt?: IsoDate;
    updatedAt?: IsoDate;
}
