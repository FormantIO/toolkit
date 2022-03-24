import { AccessLevel } from "./AccessLevel";

export interface IUserParameters {
    roles?: AccessLevel[]; // Limits which users are available for selection
}
