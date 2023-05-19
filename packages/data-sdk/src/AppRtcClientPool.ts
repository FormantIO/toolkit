import { RtcClient, SignalingPromiseClient } from "@formant/realtime-sdk";
import { RtcClientPool } from "./utils/RtcClientPool";
import { FORMANT_API_URL } from "./config";
import { Authentication } from "./Authentication";
import { defined } from "../../common/defined";

export type { PooledRtcClient } from "./utils/RtcClientPool";

const getToken = async () =>
  defined(Authentication.token, "Realtime when user isn't authorized");

const NamedRtcClientPools = {
  unknown: new RtcClientPool({
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: 0,
        receive: receiveFn,
      }),
  }),
  teleop: new RtcClientPool({
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: 1,
        receive: receiveFn,
      }),
  }),
  portForward: new RtcClientPool({
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: 2,
        receive: receiveFn,
      }),
  }),
  observe: new RtcClientPool({
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: 3,
        receive: receiveFn,
      }),
  }),
} as const;

export const AppRtcClientPool = {
  ...NamedRtcClientPools,
  ["0"]: NamedRtcClientPools.unknown,
  ["1"]: NamedRtcClientPools.teleop,
  ["2"]: NamedRtcClientPools.portForward,
  ["3"]: NamedRtcClientPools.observe,
} as const;

export function debug() {
  console.group("RtcClientPool Sizes");
  (["unknown", "teleop", "portForward", "observe"] as const).forEach((key) =>
    console.log("%s(%d)", key, NamedRtcClientPools[key].size)
  );
  console.groupEnd();
}
