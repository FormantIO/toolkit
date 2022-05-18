export interface HandheldController {
  gamepad: Gamepad;
  pulse(intensity: number, duration: number): void;
}
