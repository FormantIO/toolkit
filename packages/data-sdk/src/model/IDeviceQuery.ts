import { ITagSets } from "./ITagSets";
import { IsoDate } from "./IsoDate";
import { DeviceType } from "./DeviceType";

export interface IDeviceQuery {
  name?: string;
  query?: string;
  tags?: ITagSets;
  enabled?: boolean;
  fullyConfigured?: boolean;
  type?: DeviceType;
  count?: number;
  offset?: number;
  disabledBefore?: IsoDate;
}
