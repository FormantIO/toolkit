import { RtcClient, SignalingPromiseClient } from "@formant/realtime-sdk";

import { SessionType, SessionTypes } from "./model/SessionType";
import { RtcClientPool } from "./utils/RtcClientPool";
import { FORMANT_API_URL } from "./config";
import { Authentication } from "./Authentication";
import { defined } from "../../common/defined";

const getToken = async () =>
  defined(Authentication.token, "Realtime when user isn't authorized");

const EnumRtcClientPools = {
  [SessionTypes.UNKNOWN]: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: SessionTypes.UNKNOWN,
        receive: receiveFn,
      }),
  }),
  [SessionTypes.TELEOP]: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: SessionTypes.TELEOP,
        receive: receiveFn,
      }),
  }),
  [SessionTypes.PORT_FORWARD]: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: SessionTypes.PORT_FORWARD,
        receive: receiveFn,
      }),
  }),
  [SessionTypes.OBSERVE]: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: SessionTypes.OBSERVE,
        receive: receiveFn,
      }),
  }),
  [SessionTypes.HEADLESS]: new RtcClientPool({
    ttlMs: 2_500,
    createClient: (receiveFn) =>
      new RtcClient({
        signalingClient: new SignalingPromiseClient(FORMANT_API_URL),
        getToken,
        sessionType: SessionTypes.HEADLESS,
        receive: receiveFn,
      }),
  }),
} as const;

export const AppRtcClientPools = {
  ...EnumRtcClientPools,
  unknown: EnumRtcClientPools[SessionTypes.UNKNOWN],
  teleop: EnumRtcClientPools[SessionTypes.TELEOP],
  portForward: EnumRtcClientPools[SessionTypes.PORT_FORWARD],
  observe: EnumRtcClientPools[SessionTypes.OBSERVE],
  headless: EnumRtcClientPools[SessionTypes.HEADLESS],
} as const;

export const defaultRtcClientPool = EnumRtcClientPools[SessionTypes.TELEOP];

export const getRtcClientPool = (options: { sessionType?: SessionType }) => {
  const { sessionType } = options;

  return sessionType
    ? AppRtcClientPools[sessionType as keyof typeof AppRtcClientPools]
    : defaultRtcClientPool;
};

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
