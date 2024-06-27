import { IBaseEvent } from "./IBaseEvent";
import { IsoDate } from "./IsoDate";
import { Uuid } from "./Uuid";

export interface ITaskSummary extends IBaseEvent<"task-summary"> {
  taskSummaryFormatId: Uuid;
  report: { [key: string]: any };
  taskId: string;
  deviceId: Uuid;
  generatedAt: IsoDate;
  deletedAt?: IsoDate | null;
}
