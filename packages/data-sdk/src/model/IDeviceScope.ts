import { IScopeFilter } from "./IScopeFilter";

export interface IDeviceScope extends IScopeFilter {
  views?: IScopeFilter;
  commands?: IScopeFilter;
}
