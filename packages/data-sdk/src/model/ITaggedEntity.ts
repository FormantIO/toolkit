import { IBaseEntity } from "./IBaseEntity";
import { ITags } from "./ITags";

export interface ITaggedEntity extends IBaseEntity {
  tags: ITags;
}
