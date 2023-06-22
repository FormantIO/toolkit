import { IBaseEntity } from "./IBaseEntity";
import { Uuid } from "./Uuid";

export interface IAccount extends IBaseEntity {
  organizationId?: Uuid;
  name: string;
  parentId: Uuid | null;
}
