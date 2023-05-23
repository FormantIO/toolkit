import { RtcClient, SignalingPromiseClient } from "@formant/realtime-sdk";
import { RtcClientPool } from "./utils/RtcClientPool";
import { FORMANT_API_URL } from "./config";
import { Authentication } from "./Authentication";
import { defined } from "../../common/defined";

export enum SessionType {
  UNKNOWN = 0,
  TELEOP = 1,
  PORT_FORWARD = 2,
  OBSERVE = 3,
}

const getToken = async () =>
  defined(Authentication.token, "Realtime when user isn't authorized");

const EnumRtcClientPools = {
  [SessionType.UNKNOWN]: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: SessionType.UNKNOWN,
        receive: receiveFn,
      }),
  }),
  [SessionType.TELEOP]: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: SessionType.TELEOP,
        receive: receiveFn,
      }),
  }),
  [SessionType.PORT_FORWARD]: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: SessionType.PORT_FORWARD,
        receive: receiveFn,
      }),
  }),
  [SessionType.OBSERVE]: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: SessionType.OBSERVE,
        receive: receiveFn,
      }),
  }),
} as const;

export const AppRtcClientPools = {
  ...EnumRtcClientPools,
  unknown: EnumRtcClientPools[SessionType.UNKNOWN],
  teleop: EnumRtcClientPools[SessionType.TELEOP],
  portForward: EnumRtcClientPools[SessionType.PORT_FORWARD],
  observe: EnumRtcClientPools[SessionType.OBSERVE],
} as const;

export const defaultRtcClientPool = EnumRtcClientPools[SessionType.TELEOP];

export function debug() {
  console.group("RtcClientPool Sizes");
  console.table(
    (["unknown", "teleop", "portForward", "observe"] as const).map((key) => {
      const pool = AppRtcClientPools[key];
      return {
        name: key,
        size: pool.size,
        active: pool.isActive ? "🔥" : "🧊",
      };
    })
  );

  console.groupEnd();
}
