import { Vector3 } from "three";
import { IVector3 } from "../../../data-sdk/src/model/IVector3";

export default function vector({ x, y, z }: IVector3): Vector3 {
  return new Vector3(x, y, z);
}
