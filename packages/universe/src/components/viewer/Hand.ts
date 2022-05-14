import { Group, Object3D, Vector3 } from "three";

export type Joints =
  | "wrist"
  | "thumb-metacarpal"
  | "thumb-phalanx-proximal"
  | "thumb-phalanx-distal"
  | "thumb-tip"
  | "index-finger-metacarpal"
  | "index-finger-phalanx-proximal"
  | "index-finger-phalanx-intermediate"
  | "index-finger-phalanx-distal"
  | "index-finger-tip"
  | "middle-finger-metacarpal"
  | "middle-finger-phalanx-proximal"
  | "middle-finger-phalanx-intermediate"
  | "middle-finger-phalanx-distal"
  | "middle-finger-tip"
  | "ring-finger-metacarpal"
  | "ring-finger-phalanx-proximal"
  | "ring-finger-phalanx-intermediate"
  | "ring-finger-phalanx-distal"
  | "ring-finger-tip"
  | "pinky-finger-metacarpal"
  | "pinky-finger-phalanx-proximal"
  | "pinky-finger-phalanx-intermediate"
  | "pinky-finger-phalanx-distal"
  | "pinky-finger-tip";

export interface IJoint {
  visible: boolean;
  jointRadius: number;
}

export interface IHandController {
  visible: boolean;
  joints: { [key in Joints]: Group & IJoint };
}

// typings for XRHandMeshModel
export type HandController = Object3D & IHandController;

export interface IHand {
  controller: HandController;
  getPointerPosition(): Vector3 | null;
  getJoint(joint: Joints): Object3D | null;
  intersectBoxObject(boxObject: Object3D): boolean;
}

// typings for OculusHandModel
export type Hand = Object3D & IHand;
