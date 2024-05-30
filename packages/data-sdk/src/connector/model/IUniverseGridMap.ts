import { ITransform } from "../../model/ITransform";

export interface IUniverseGridMap {
  worldToLocal?: ITransform;
  width: number;
  height: number;
  resolution: number;
  origin: ITransform;
  data: number[];
  alpha: number[];
}
