import { Uuid } from "./Uuid";

export interface ITaggedUsers {
    [key: string]: Uuid[];
}
