import { EventSortableColumn } from "./EventSortableColumn";
import { SortOrder } from "./SortOrder";

export interface IEventSort {
  column: EventSortableColumn;
  order: SortOrder;
}
