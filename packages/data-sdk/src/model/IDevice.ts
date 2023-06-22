import { ITaggedEntity } from "./ITaggedEntity";
import { Uuid } from "./Uuid";
import { IScopeFilter } from "./IScopeFilter";
import { IsoDate } from "./IsoDate";
import { DeviceType } from "./DeviceType";
import { IDeviceScope } from "./IDeviceScope";
import { IDeviceState } from "./IDeviceState";
import { IDeviceFollower } from "./IdeviceFollower";

export interface IDevice extends ITaggedEntity {
  organizationId?: Uuid;
  name: string;
  type?: DeviceType;
  userId?: Uuid | null;
  fleetId?: Uuid | null;
  eventTriggerGroupId?: Uuid | null;
  scope?: IScopeFilter | null;
  deviceScope?: IDeviceScope;
  publicKey: string;
  desiredAgentVersion?: string | null;
  desiredConfigurationVersion?: number | null;
  temporaryConfigurationVersion?: number | null;
  temporaryConfigurationExpiration?: IsoDate | null;
  temporaryConfigurationTemplateId?: Uuid | null;
  followers?: IDeviceFollower[];
  slackChannels?: string[];
  phoneNumber?: string;
  state?: IDeviceState;
  enabled?: boolean;
  fullyConfigured?: boolean;
  disabledAt?: IsoDate | null;
}
