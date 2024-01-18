import { IPoseWithCovariance } from "../../../data-sdk/src/model/IPoseWithCovariance";
import { ITransform } from "../main";

export interface IUniverseOdometry extends IPoseWithCovariance {
  worldToLocal?: ITransform;
}
