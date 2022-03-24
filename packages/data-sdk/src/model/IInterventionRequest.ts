import { IBaseEvent } from "./IBaseEvent";
import { IInterventionResponse } from "./IInterventionResponse";
import { IInterventionTypeMap } from "./IInterventionTypeMap";
import { InterventionType } from "./InterventionType";

export interface IInterventionRequest<
    T extends InterventionType = InterventionType
> extends IBaseEvent<"intervention-request"> {
    interventionType: T;
    data: IInterventionTypeMap[T]["request"];
    responses?: IInterventionResponse<T>[];
}
