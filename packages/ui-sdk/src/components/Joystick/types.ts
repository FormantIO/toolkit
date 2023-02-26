export const teleopJoystickPositions = ["left", "right"] as const;
export const twistDimensions = [
  "linear-x",
  "linear-y",
  "linear-z",
  "angular-x",
  "angular-y",
  "angular-z",
] as const;

export type TeleopJoystickPosition = typeof teleopJoystickPositions[number];

export type TwistDimension = typeof twistDimensions[number];

export interface ITeleopJoystickAxisConfiguration {
  dimension?: TwistDimension;
  scale?: number;
  expo?: number;
  gamepadAxis?: number;
}

export interface ITeleopJoystickConfiguration {
  newValueDebounce?: number;
  sameValueDebounce?: number;
  disableTriggerJoystickMapping?: boolean;
  disableGamepadAndKeyboard?: boolean;
  position?: TeleopJoystickPosition;
  x?: ITeleopJoystickAxisConfiguration;
  y?: ITeleopJoystickAxisConfiguration;
}

export type ITeleopTwistValue = {
  dimension: TwistDimension;
  value: number;
};

export interface IVector2 {
  x: number;
  y: number;
}

export interface IShutdownPromiseInterval {
  stop(): Promise<void>;
}

export interface IPromiseIntervalOptions {
  func: () => Promise<void>;
  delay: number;
  immediate?: boolean;
}

export interface IShutdownPromiseInterval {
  stop(): Promise<void>;
}

export type Timeout = ReturnType<typeof setTimeout>;
