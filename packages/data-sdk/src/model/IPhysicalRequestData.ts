import { IPhysicalInterventionMetadata } from "./IPhysicalInterventionMetadata";
import { IsoDate } from "./IsoDate";

export interface IPhysicalRequestData {
  reason: string;
  metadata?: IPhysicalInterventionMetadata;
  startTime: IsoDate;
  endTime: IsoDate;
}
