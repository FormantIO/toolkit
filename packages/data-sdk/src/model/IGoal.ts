import { ITransform } from "./ITransform";

export interface IGoal {
    worldToLocal: ITransform;
    pose: ITransform;
}
