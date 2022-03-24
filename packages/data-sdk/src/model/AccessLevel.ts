import { accessLevels } from "./accessLevels";

export type AccessLevel = typeof accessLevels[number];

export const viewer = "viewer" as AccessLevel;
export const operator = "operator" as AccessLevel;
export const administrator = "administrator" as AccessLevel;
