import { ITaggedEntity } from "./ITaggedEntity";
import { Policies } from "./Policies";
import { Uuid } from "./Uuid";

export interface IRole extends ITaggedEntity {
  organizationId?: Uuid;
  name: string;
  policies: Policies;
  isImmutable?: boolean;
}
