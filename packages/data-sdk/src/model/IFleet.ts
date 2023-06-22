import { IDeviceScope } from "./IDeviceScope";
import { ITaggedEntity } from "./ITaggedEntity";
import { Uuid } from "./Uuid";

export interface IFleet extends ITaggedEntity {
  organizationId?: Uuid;
  name: string;
  scope?: IDeviceScope;
}
