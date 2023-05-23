import { IRtcClientConfiguration } from "@formant/realtime-sdk";

export type SessionType = NonNullable<IRtcClientConfiguration["sessionType"]>;

export const SessionTypes = {
  UNKNOWN: 0,
  TELEOP: 1,
  PORT_FORWARD: 2,
  OBSERVE: 3,
} as const satisfies Record<string, SessionType>;

// For backwards-compatibility
export const SessionTypeConstants = {
  ...SessionTypes,

  Unknown: SessionTypes.UNKNOWN,
  Teleop: SessionTypes.TELEOP,
  PortForward: SessionTypes.PORT_FORWARD,
  Observe: SessionTypes.OBSERVE,

  unknown: SessionTypes.UNKNOWN,
  teleop: SessionTypes.TELEOP,
  portForward: SessionTypes.PORT_FORWARD,
  observe: SessionTypes.OBSERVE,
} as const satisfies Record<string, SessionType>;
