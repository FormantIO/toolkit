import { Group, Raycaster, XRHandedness } from "three";

export interface IController {
  raycaster: Raycaster;
  handedness: XRHandedness;
  pulse(intensity: number, duration: number): void;
}
export type Controller = Group & IController;
