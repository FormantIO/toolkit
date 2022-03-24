import { ITransform } from "./ITransform";

export interface IPath {
    worldToLocal: ITransform;
    poses: ITransform[];
}
