import { AggregateLevel } from "./AggregateLevel";
import { IFilter } from "./IFilter";
import { IsoDate } from "./IsoDate";

export interface IQuery extends IFilter {
  aggregate?: AggregateLevel;
  start: IsoDate;
  end: IsoDate;
  latestOnly?: boolean;
}
