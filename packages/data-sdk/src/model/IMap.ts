import { ITransform } from "./ITransform";

export interface IMap {
    url: string;
    size?: number;
    width: number;
    height: number;
    resolution: number;
    origin: ITransform;
    worldToLocal: ITransform;
}
