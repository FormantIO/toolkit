// @ts-ignore-next-line

export type Field = "x" | "y" | "z" | "rgb" | "rgba" | "intensity";
export type Type = "i" | "u" | "f";
export type Data = "ascii" | "binary" | "binary_compressed";

export interface IPcdHeader {
  version: string;
  fields: Field[];
  size: number[];
  type: Type[];
  count: number[];
  height: number;
  width: number;
  points: number;
  data: Data;
}

export interface IPcd {
  header: IPcdHeader;
  positions?: Float32Array | number[];
  colors?: Float32Array | number[];
  intensity?: Float32Array | number[];
}
