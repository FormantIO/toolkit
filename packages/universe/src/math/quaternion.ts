import { Quaternion } from "three";
import { IQuaternion } from "../../../data-sdk/src/model/IQuaternion";

export function quaternion({ x, y, z, w }: IQuaternion): Quaternion {
  return new Quaternion(x, y, z, w);
}
