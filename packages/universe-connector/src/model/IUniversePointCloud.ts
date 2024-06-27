import { ITransform } from "@formant/data-sdk";
import { IPcd } from "./IPcd";

export interface IUniversePointCloud {
  pcd?: IPcd;
  worldToLocal?: ITransform;
}
