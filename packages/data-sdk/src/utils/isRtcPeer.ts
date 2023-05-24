import { IRtcPeer } from "@formant/realtime-sdk/dist/model/IRtcPeer";

export const isRtcPeer = (peer: IRtcPeer | undefined): peer is IRtcPeer =>
  peer !== undefined &&
  peer.capabilities !== undefined &&
  peer.capabilitySet !== undefined;
