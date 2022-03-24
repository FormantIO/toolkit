import { AnnotationAreaType } from "./AnnotationAreaType";
import { HexRgbColor } from "./HexRgbColor";
import { IAnnotationAreaTypeMap } from "./IAnnotationAreaTypeMap";

export interface IImageAnnotation<
  T extends AnnotationAreaType = AnnotationAreaType
> {
  label: string;
  type: AnnotationAreaType;
  area: IAnnotationAreaTypeMap[T];
  color: HexRgbColor;
}
