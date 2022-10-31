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

export enum ACTIONS {
  SET_NUMBER,
  SET_TYPE,
  SET_PERIOD,
  SET_STREAM_NAME,
  SET_STREAM_KEY,
  LOAD_CONFIGURATION,
}

export interface IActions {
  type: ACTIONS;
  payload: {
    value:
      | string
      | number
      | AggregatePeriod
      | AggregateType
      | IAggregateConfiguration;
  };
}
