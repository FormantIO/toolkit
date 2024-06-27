import { IPoseWithCovariance } from "../../model/IPoseWithCovariance";
import { ITransform } from "../../model/ITransform";

export interface IUniverseOdometry extends IPoseWithCovariance {
  worldToLocal?: ITransform;
}
