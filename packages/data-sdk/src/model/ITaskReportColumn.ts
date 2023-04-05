import { ISqlColumn } from "./ISqlColumn";

export interface ITaskReportColumn {
  tableName: string;
  columns?: ISqlColumn[];
  name: string;
  yAxis?: string;
}
