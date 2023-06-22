import { IOnDemandStreamPresence } from "./IOnDemandStreamPresence";
import { OnDemandBufferType } from "./OnDemandBufferType";

export interface IOnDemandBuffer {
  bufferType: OnDemandBufferType;
  streams: IOnDemandStreamPresence[];
}
