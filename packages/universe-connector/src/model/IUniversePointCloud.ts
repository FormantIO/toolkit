import { ITransform } from "../main";
import { IPcd } from "./IPcd";

export interface IUniversePointCloud {
  pcd?: IPcd;
  worldToLocal?: ITransform;
}
