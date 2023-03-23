export type DataType = "text" | "numeric" | "bitset" | "numeric set";

type Point = [number, any];

export interface IStream<T extends DataType> {
  agentId: string;
  deviceId: string;
  name: string;
  points: Point[];
  tags: { [key: string]: string };
  type: T;
}

interface DataTypeBase<T extends DataType> {
  deviceId: string;
  id: string;
  streamName: string;
  streamType: T;
  currentValueTime: number;
  tags?: { [key: string]: string };
}
export interface Bitset extends DataTypeBase<"bitset"> {
  currentValue: {
    keys: string[];
    values: boolean[];
  };
}

export interface Text extends DataTypeBase<"text"> {
  currentValue: string;
}

export interface Numeric extends DataTypeBase<"numeric"> {
  currentValue: number;
}

export interface Iconfiguration {
  name: string;
  fullWidth: boolean;
  streamType: DataType;
  expectedValue: any;
}

export interface lastKnowValue {
  currentValue: string | number | boolean;
  streamType: "numeric" | "text" | "bitset";
}

export type Configuration = {
  streams: Iconfiguration[];
};

export interface ICurrentValues {
  [key: string]: string | number | boolean;
}
export interface INumericConfiguration {
  name: string;
  greaterThan: number;
  lesserThan: number;
}

export interface InumericExpectedValue {
  greaterThan: number;
  lesserThan: number;
}

export interface ITextConfiguration {
  name: string;
  expectedValue: string;
}

export interface IBitsetConfiguration {
  name: string;
  expectedValue: IBit[];
}

interface IBit {
  key: string;
  value: boolean;
}

export interface IReducedConfiguration {
  [key: string]: string | boolean | InumericExpectedValue;
}

const streamTypes = ["textStreams", "bitsetStreams", "numericStreams"] as const;

export type StreamConfigurationType = typeof streamTypes[number];
export interface IConfiguration {
  fullScreenMode: boolean;
  rowHeight?: number;
  fontSize?: number;
  textStreams: ITextConfiguration[];
  numericStreams: INumericConfiguration[];
  bitsetStreams: IBitsetConfiguration[];
}

export type ConfigurationTypes =
  | ITextConfiguration
  | IBitsetConfiguration
  | INumericConfiguration;

export type Status = "warning" | "good" | "offline";
