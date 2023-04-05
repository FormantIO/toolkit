import { IBaseEntity } from "./IBaseEntity";
import { Uuid } from "./Uuid";
import { IScopeFilter } from "./IScopeFilter";
import { IsoDate } from "./IsoDate";

export interface IShare extends IBaseEntity {
  organizationId?: Uuid;
  userId?: Uuid;
  code?: string;
  scope: IScopeFilter;
  time: IsoDate;
  expiration?: IsoDate | null;
  channelId?: Uuid;
  message?: string;
  userName?: string;
  delegateTeleop?: boolean;
}
