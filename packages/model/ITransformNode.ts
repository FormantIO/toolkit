import { ITransform } from "./ITransform";

export interface ITransformNode {
    name?: string;
    transform?: ITransform;
    children?: ITransformNode[];
    url?: string;
}
