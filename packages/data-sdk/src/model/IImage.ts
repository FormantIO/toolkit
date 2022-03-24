import { IImageAnnotation } from "./IImageAnnotation";

export interface IImage {
  url: string;
  size?: number;
  annotations?: IImageAnnotation[];
}
