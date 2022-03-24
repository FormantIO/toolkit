import { StreamType } from "./StreamType";
import { ITagSets } from "./ITagSets";
import { Uuid } from "./Uuid";

export interface IFilter {
  deviceIds?: Uuid[];
  names?: string[];
  types?: StreamType[];
  tags?: ITagSets;
  notNames?: string[];
}
