import { ITransform } from "../../model/ITransform";

export interface IUniverseGridMap {
  worldToLocal?: ITransform;
  width: number;
  height: number;
  resolution: number;
  origin: ITransform;
  url?: string;
  data?: number[];
  alpha?: number[];
}
