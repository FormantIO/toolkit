import { IBaseEvent } from "./IBaseEvent";
import { IInterventionTypeMap } from "./IInterventionTypeMap";
import { InterventionType } from "./InterventionType";
import { Uuid } from "./Uuid";

export interface IInterventionResponse<
  T extends InterventionType = InterventionType
> extends IBaseEvent<"intervention-response"> {
  userId?: Uuid;
  interventionId: Uuid;
  interventionType: T;
  data: IInterventionTypeMap[T]["response"];
}
