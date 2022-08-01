type DataType = "text" | "numeric" | "bitset";

interface DataTypeBase<T extends DataType> {
  deviceId: string;
  id: string;
  streamName: string;
  streamType: string;
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

export type configuration = {
  streamName: string;
  enable: boolean;
};

export interface lastKnowValue {
  currentValue: string | number | boolean;
  streamType: "numeric" | "text" | "bitset";
}
