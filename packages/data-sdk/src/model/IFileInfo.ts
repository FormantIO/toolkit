import { Uuid } from "./Uuid";

export interface IFileInfo {
    id: Uuid;
    name?: string;
    url?: string;
}
