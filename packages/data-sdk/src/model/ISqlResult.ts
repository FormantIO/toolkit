import { IAggregateRow } from "./IAggregateRow";
import { ISqlColumn } from "./ISqlColumn";
import { ISqlRow } from "./ISqlRow";

export interface ISqlResult {
  rows: ISqlRow[];
  columns: ISqlColumn[];
  aggregates?: IAggregateRow[];
  sqlText: string;
  aggregateSqlText?: string;
  rowCount: number;
  unit?: string;
}
