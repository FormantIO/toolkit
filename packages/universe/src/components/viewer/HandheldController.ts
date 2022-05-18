export interface HandheldController {
  gamepad: Gamepad;
  vibrate(params: {
    startDelay: number;
    duration: number;
    weakMagnitude: number;
    strongMagnitude: number;
  }): void;
}
