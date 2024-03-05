import { IPoseWithCovariance } from "../../../data-sdk/src/model/IPoseWithCovariance";
import { ITransform } from "@formant/data-sdk";

export interface IUniverseOdometry extends IPoseWithCovariance {
  worldToLocal?: ITransform;
}
