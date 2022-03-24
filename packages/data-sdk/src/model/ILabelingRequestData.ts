import { ILabel } from "./ILabel";
import { ILabeledPolygon } from "./ILabeledPolygon";

export interface ILabelingRequestData {
    instruction: string;
    imageUrl: string;
    labels: ILabel[];
    hint?: ILabeledPolygon[];
}
