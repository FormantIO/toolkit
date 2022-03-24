import { IEventFilter } from "./IEventFilter";
import { IEventSort } from "./IEventSort";

export interface IEventQuery extends IEventFilter {
  offset?: number;
  count?: number;
  sort?: IEventSort;
}
