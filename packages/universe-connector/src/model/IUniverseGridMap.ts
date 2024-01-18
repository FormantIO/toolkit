import { ITransform } from "../../../model/ITransform";

export interface IUniverseGridMap {
  canvas: HTMLCanvasElement;
  worldToLocal?: ITransform;
  width: number;
  height: number;
  resolution: number;
  origin: ITransform;
  data: number[];
}
