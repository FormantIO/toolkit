import * as THREE from "three";
import { getDefaultContext } from "./DeviceContext";
import { LoadingManager } from "three";
import URDFLoader from "urdf-loader";
import { Object3D } from "three";

export class DeviceURDF extends THREE.Object3D {
  constructor(_context = getDefaultContext()) {
    super();
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);
    loader.load(
      "https://cdn.jsdelivr.net/gh/gkjohnson/urdf-loaders@0.10.2/urdf/T12/urdf/T12.URDF",
      (robot) => {
        this.add(robot as unknown as Object3D);
      }
    );
  }
}
