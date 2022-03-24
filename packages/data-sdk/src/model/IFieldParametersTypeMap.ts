import { ISheetParameters } from "./ISheetParameters";
import { ITagParameters } from "./ITagParameters";
import { IUserParameters } from "./IUserParameters";

export interface IFieldParametersTypeMap {
  tag: ITagParameters;
  sheet: ISheetParameters;
  user: IUserParameters;
}
