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
  expectedValue: any
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
