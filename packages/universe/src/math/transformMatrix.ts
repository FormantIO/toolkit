import { Matrix4 } from "three";
import { ITransform } from "../../../data-sdk/src/model/ITransform";
import { quaternion } from "./quaternion";
import { vector } from "./vector";

export function transformMatrix({
  translation,
  rotation,
}: ITransform): Matrix4 {
  return new Matrix4()
    .multiply(new Matrix4().setPosition(vector(translation)))
    .multiply(new Matrix4().makeRotationFromQuaternion(quaternion(rotation)));
}
