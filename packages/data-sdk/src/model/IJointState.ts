import { ITransform } from "./ITransform";

export interface IJointState {
  world_to_local?: ITransform;
  name: string[];
  position: number[];
  velocity?: number[];
  effort?: number[];
}
