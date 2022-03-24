import { IBaseEntity } from "./IBaseEntity";
import { IInterventionTypeMap } from "./IInterventionTypeMap";
import { InterventionType } from "./InterventionType";
import { Uuid } from "./Uuid";

export interface IInterventionResponse<
    T extends InterventionType = InterventionType
> extends IBaseEntity {
    userId?: Uuid;
    interventionId: Uuid;
    interventionType: T;
    data: IInterventionTypeMap[T]["response"];
}
