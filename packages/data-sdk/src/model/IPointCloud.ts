import { ITransform } from "./ITransform";

export interface IPointCloud {
    url: string;
    size?: number;
    worldToLocal?: ITransform;
}
