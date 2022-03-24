import { ITransform } from "./ITransform";
import { ITwist } from "./ITwist";

export interface IOdometry {
    pose: ITransform;
    twist: ITwist;
    worldToLocal: ITransform;
}
