import { RtcClient, SignalingPromiseClient } from "@formant/realtime-sdk";
import { RtcClientPool } from "./utils/RtcClientPool";
import { FORMANT_API_URL } from "./config";
import { Authentication } from "./Authentication";
import { defined } from "../../common/defined";

const getToken = async () =>
  defined(Authentication.token, "Realtime when user isn't authorized");

const NamedRtcClientPools = {
  unknown: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: 0,
        receive: receiveFn,
      }),
  }),
  teleop: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: 1,
        receive: receiveFn,
      }),
  }),
  portForward: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: 2,
        receive: receiveFn,
      }),
  }),
  observe: new RtcClientPool({
    ttlMs: 2_500,
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
  console.table(
    (["unknown", "teleop", "portForward", "observe"] as const).map((key) => {
      const pool = NamedRtcClientPools[key];
      return {
        name: key,
        size: pool.size,
        active: pool.isActive ? "🔥" : "🧊",
      };
    })
  );

  console.groupEnd();
}
