import { ITransform } from "../../../data-sdk/src/model/ITransform";

export interface IRtcPointCloud {
  world_to_local?: ITransform;
  data: string;
}
