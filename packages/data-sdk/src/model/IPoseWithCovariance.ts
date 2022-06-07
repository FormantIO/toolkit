import { ITransform } from "./ITransform";

export interface IPoseWithCovariance {
  pose: ITransform;
  // Should have length of 36
  // Can be empty, will be filled by rectifyZeroValues
  covariance: Array<number>;
}
