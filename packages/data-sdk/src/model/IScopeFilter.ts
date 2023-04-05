import { IFilter } from "./IFilter";
import { IsoDate } from "./IsoDate";

export interface IScopeFilter extends IFilter {
  start?: IsoDate;
  end?: IsoDate;
}
