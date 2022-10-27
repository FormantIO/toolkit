export type AggregateType =
  | "min"
  | "max"
  | "standard deviation"
  | "average"
  | "sum"
  | "count";

export type AggregatePeriod = "day" | "week" | "month";

export interface IAggregateConfiguration {
  aggregateType: AggregateType;
  streamName: string;
  numericSetKey: string;
  aggregateBy: AggregatePeriod;
  numAggregates: number;
}
