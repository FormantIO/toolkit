export type AggregateType =
  | "min"
  | "max"
  | "standard deviation"
  | "average"
  | "sum"
  | "count";

export type AggregatePeriod = "day" | "week" | "month";

type StreamType = "numeric" | "numeric set";

export interface IConfiguration<T extends StreamType> {
  fullScreenMode: boolean;
  streamType: T;
  aggregateType: AggregateType;
  aggregateBy: AggregatePeriod;
  numAggregates: number;
  deviceIds?: string[];
  tags?: ITag[];
}

export interface ITag {
  key: string;
  value: string;
}

export interface INumericConfiguration extends IConfiguration<"numeric"> {
  numericStream: string;
}

export interface INumericSetConfiguration
  extends IConfiguration<"numeric set"> {
  numericSetStream: string;
  numericSetKey: string;
}
export type ConfigurationTypes =
  | INumericConfiguration
  | INumericSetConfiguration;
