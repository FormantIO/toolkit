import { IQuaternion } from "./IQuaternion";
import { IVector3 } from "./IVector3";

export interface ITransform {
    translation: IVector3;
    rotation: IQuaternion;
}
