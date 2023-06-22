import { IScopeFilter } from "./IScopeFilter";
import { RequireKeys } from "./RequiredKeys";
export interface IUserResourceScope {
  roles?: IScopeFilter;
  users?: IScopeFilter;
  teams?: IScopeFilter;
  devices?: IScopeFilter;
  fleets?: IScopeFilter;
  events?: IScopeFilter;
  views?: IScopeFilter;
}

export interface IUserScope extends IScopeFilter, IUserResourceScope {}

export type IFullUserScope = IScopeFilter & RequireKeys<IUserResourceScope>;

export const fullAccessUserScope: IFullUserScope = {
  roles: {},
  users: {},
  teams: {},
  devices: {},
  fleets: {},
  events: {},
  views: {},
};
