import { IBaseEntity } from "./IBaseEntity";
import { Uuid } from "./Uuid";
import { ISqlQuery } from "./ISqlQuery";
import { IAnalyticsModuleConfiguration } from "./IAnalyticsModuleConfiguration";

export interface IAnalyticsModule extends IBaseEntity {
  id?: Uuid;
  organizationId?: Uuid;
  name: string;
  query: ISqlQuery | null;
  configuration: IAnalyticsModuleConfiguration | null;
  layout: any | null;
  data: any[] | null;
  streamIds: Uuid[];
  fullscreen?: boolean;
}
