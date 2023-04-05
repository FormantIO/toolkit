import { AnalyticsAggregateType } from "./AnalyticsAggregateType";

export interface IAggregateRow {
  unit?: string;
  label?: string;
  name: string;
  value: number;
  type: AnalyticsAggregateType;
}
