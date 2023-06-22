import { IOnDemandPresenceStreamItemGroup } from "./IOnDemandPresenceStreamItemGroup";

export interface IOnDemandStreamPresence {
  streamName: string;
  presence: IOnDemandPresenceStreamItemGroup[];
}
