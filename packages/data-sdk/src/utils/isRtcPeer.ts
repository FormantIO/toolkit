import { IRtcPeer } from "@formant/realtime-sdk";

export const isRtcPeer = (peer: IRtcPeer | undefined): peer is IRtcPeer =>
  peer !== undefined &&
  peer.capabilities !== undefined &&
  peer.capabilitySet !== undefined;
