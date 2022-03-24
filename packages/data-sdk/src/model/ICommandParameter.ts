import { IConfigurationMap } from "./IConfigurationMap";
import { IFileInfo } from "./IFileInfo";
import { IsoDate } from "./IsoDate";

export interface ICommandParameter {
  value?: string;
  meta?: IConfigurationMap;
  scrubberTime: IsoDate;
  files?: IFileInfo[];
}
