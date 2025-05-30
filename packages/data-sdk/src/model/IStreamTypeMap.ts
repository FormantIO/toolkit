import { IBattery } from "./IBattery";
import { IBitset } from "./IBitset";
import { IFile } from "./IFile";
import { IHealth } from "./IHealth";
import { IImage } from "./IImage";
import { ILocalization } from "./ILocalization";
import { ILocation } from "./ILocation";
import { INumericSetEntry } from "./INumericSetEntry";
import { IPointCloud } from "./IPointCloud";
import { ITransformNode } from "./ITransformNode";
import { IVideo } from "./IVideo";

// NOTE: Also add an entry in streamTypes

export interface IStreamTypeMap {
  bitset: IBitset;
  localization: ILocalization;
  "point cloud": IPointCloud;
  location: ILocation;
  file: IFile;
  health: IHealth;
  "transform tree": ITransformNode;
  battery: IBattery;
  video: IVideo;
  "numeric set": INumericSetEntry[];
  json: string;
  image: IImage;
  numeric: number;
  text: string;
}
