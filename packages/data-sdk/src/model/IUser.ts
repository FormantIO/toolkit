import { ITaggedEntity } from "./ITaggedEntity";
import { Uuid } from "./Uuid";
import { IsoDate } from "./IsoDate";
import { CognitoRegion } from "./CognitoRegion";
import { IUserScope } from "./IUserScope";

export interface IUser extends ITaggedEntity {
  organizationId?: Uuid;
  accountId?: Uuid | null;
  roleId: Uuid | null;
  email?: string;
  firstName?: string;
  lastName?: string;
  scope?: IUserScope | null;
  teamId?: Uuid | null;
  eventTriggerGroupId?: Uuid | null;
  phoneNumber?: string;
  enabled?: boolean;
  isOrganizationOwner?: boolean;
  termsAccepted?: string;
  lastLoggedIn?: IsoDate;
  passwordHash?: string | null;
  isSingleSignOn?: boolean;
  isServiceAccount?: boolean;
  region: CognitoRegion;
  jobTitle?: string;
  language?: string;
  units?: string;
  timezone?: string;
}
