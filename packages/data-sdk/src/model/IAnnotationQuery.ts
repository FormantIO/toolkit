import { IEventQuery } from "./IEventQuery";
import { AggregateLevel } from "./AggregateLevel";

export interface IAnnotationQuery extends IEventQuery {
  tagKey?: string;
  annotationName?: string;
  aggregate?: AggregateLevel;
}
