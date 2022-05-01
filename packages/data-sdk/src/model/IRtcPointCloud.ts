import { ITransform } from "./ITransform";

export interface IRtcPointCloud {
  world_to_local?: ITransform;
  data: string;
}
