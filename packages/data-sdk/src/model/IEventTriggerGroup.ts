import { Uuid } from "./Uuid";
import { IScopeFilter } from "./IScopeFilter";
import { ITaggedEntity } from "./ITaggedEntity";
import { ITags } from "./ITags";
import { IDevice } from "./IDevice";

export interface IEventTriggerGroup extends ITaggedEntity {
  organizationId?: Uuid;
  deviceScope: IScopeFilter;
  enabled?: boolean;
  smsTags: ITags;
  phoneNumbers?: string[];
  devices?: IDevice[];
}
