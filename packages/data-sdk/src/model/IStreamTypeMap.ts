import { IBattery } from "./IBattery";
import { IBitset } from "./IBitset";
import { IFile } from "./IFile";
import { IHealth } from "./IHealth";
import { IImage } from "./IImage";
import { IJoy } from "./IJoy";
import { ILocalization } from "./ILocalization";
import { ILocation } from "./ILocation";
import { INumericSetEntry } from "./INumericSetEntry";
import { IPointCloud } from "./IPointCloud";
import { ITransformNode } from "./ITransformNode";
import { ITwist } from "./ITwist";
import { IVideo } from "./IVideo";

// NOTE: Also add an entry in streamTypes

export interface IStreamTypeMap {
  twist: ITwist;
  joy: IJoy;
  boolean: boolean;
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
