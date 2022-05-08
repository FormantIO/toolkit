import { Group } from "three";

export interface IController {
  pulse(intensity: number, duration: number): void;
}
export type Controller = Group & IController;
