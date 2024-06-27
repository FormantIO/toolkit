import { ITransform } from "../../model/ITransform";
import { IPcd } from "./IPcd";

export interface IUniversePointCloud {
  pcd?: IPcd;
  worldToLocal?: ITransform;
}
