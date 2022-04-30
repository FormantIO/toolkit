import { IColorRGBA } from "./IColorRGBA";
import { ITransform } from "./ITransform";
import { IVector3 } from "./IVector3";
import { Marker3DAction } from "./Marker3DAction";
import { Marker3DType } from "./Marker3DType";

export interface IMarker3D {
    world_to_local?: ITransform;
    ns: string;
    id: number;
    type: Marker3DType;
    action: Marker3DAction;
    pose: ITransform;
    scale: IVector3;
    color: IColorRGBA;
    lifetime: number;
    frame_id: string;
    frame_locked: boolean;
    points: IVector3[];
    colors: IColorRGBA[];
    text: string;
    mesh_resource: string;
    mesh_use_embedded_materials: boolean;
}
