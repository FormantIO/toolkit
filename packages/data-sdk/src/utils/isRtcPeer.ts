import { IRtcPeer } from "@formant/realtime-sdk/dist/model/IRtcPeer";

export const isRtcPeer = (_: IRtcPeer | any): _ is IRtcPeer =>
  _.capabilities !== undefined && _.capabilitySet !== undefined;
