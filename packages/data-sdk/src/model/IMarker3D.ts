import { IColorRGBA } from "./IColorRGBA";
import { IQuaternion } from "./IQuaternion";
import { ITransform } from "./ITransform";
import { IVector3 } from "./IVector3";

export interface IMarker3D {
  world_to_local?: {
    position: IVector3;
    orientation: IQuaternion;
  };
  ns: string;
  id: number;
  type: number;
  action: number;
  pose: {
    position: IVector3;
    orientation: IQuaternion;
  };
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
