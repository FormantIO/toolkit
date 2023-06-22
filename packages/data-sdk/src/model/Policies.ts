import { AccessLevel, administrator, operator, viewer } from "./AccessLevel";
import { Resource } from "./Resource";

const editTags = "edit-tags" as const;

export const resourcePolicyMap = {
  organization: [administrator, viewer] as const,
  users: [administrator, viewer] as const,
  devices: [administrator, viewer, editTags] as const,
  roles: [administrator, viewer, editTags] as const,
  billing: [administrator, viewer] as const,
  streams: [administrator, viewer] as const,
  views: [administrator, viewer, editTags] as const,
  channels: [administrator, viewer] as const,
  comments: [administrator, operator, viewer] as const,
  teleop: [administrator, operator] as const,
  terminal: [administrator, operator] as const,
  portForwarding: [administrator, operator] as const,
  events: [administrator, viewer] as const,
  commands: [administrator, operator, viewer] as const,
  annotations: [administrator, operator, viewer] as const,
  capture: [administrator, operator] as const,
  share: [administrator, operator] as const,
  fileStorage: [administrator, operator] as const,
  integrations: [administrator, operator, viewer] as const,
  interventions: [administrator, viewer] as const,
  analytics: [administrator, viewer] as const,
  keyValueStorage: [administrator, operator, viewer] as const,
  schedules: [administrator, viewer] as const,
  kiosk: [administrator, viewer] as const,
  taskSummaries: [administrator, operator] as const,
};

export type ResourcePolicy<T extends Resource> =
  (typeof resourcePolicyMap)[T][number];

export type Policies = {
  [_ in Resource]?: ResourcePolicy<_>[];
};

export function getAccessLevel(
  policies: ResourcePolicy<Resource>[]
): AccessLevel | undefined {
  if (policies.includes(administrator)) {
    return administrator;
  }
  if (policies.includes(operator)) {
    return operator;
  }
  if (policies.includes(viewer)) {
    return viewer;
  }
  return undefined;
}

export function includesAccess<T extends Resource = Resource>(
  accessPolicy: ResourcePolicy<T>[] | undefined,
  targetPolicy: ResourcePolicy<T>
): boolean {
  if (!accessPolicy || accessPolicy.length === 0) {
    return false;
  }

  switch (targetPolicy) {
    case "viewer":
      return accessPolicy.some(
        (_) => _ === "administrator" || _ === "operator" || _ === "viewer"
      );
    case "operator":
      return accessPolicy.some(
        (_) => _ === "administrator" || _ === "operator"
      );
    case "administrator":
      return accessPolicy.some((_) => _ === "administrator");
    default:
      return accessPolicy.some(
        (_) => _ === "administrator" || _ === targetPolicy
      );
  }
}
