import { IsoDate } from "./IsoDate";
import { IFilter } from "./IFilter";
import { AggregateLevel } from "./AggregateLevel";
import { IStreamColumn } from "./IStreamColumn";
import { ITaskReportColumn } from "./ITaskReportColumn";

export interface ISqlQuery extends IFilter {
  sqlQuery?: string;
  parameters?: string[];
  start?: IsoDate;
  end?: IsoDate;
  streamColumns?: IStreamColumn[];
  taskColumns?: ITaskReportColumn[];
  aggregateLevel?: AggregateLevel;
  limit?: number;
  orderByColumn?: string;
  orderByDescending?: boolean;
  visibleColumns?: string[];
  filters?: string[];
  type?: "stream" | "task" | "advanced";
  unit?: string;
}
