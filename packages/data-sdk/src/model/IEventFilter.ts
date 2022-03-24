import { EventType } from "./EventType";
import { IFilter } from "./IFilter";
import { IsoDate } from "./IsoDate";
import { Severity } from "./Severity";
import { Uuid } from "./Uuid";

export interface IEventFilter extends IFilter {
  id?: Uuid;
  viewed?: boolean;
  keyword?: string;
  message?: string;
  start?: IsoDate;
  end?: IsoDate;
  eventTypes?: EventType[];
  notificationEnabled?: boolean;
  userIds?: Uuid[];
  annotationTemplateIds?: Uuid[];
  disableNullMatches?: boolean;
  severities?: Severity[];
}
