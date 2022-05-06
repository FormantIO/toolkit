import { UniverseLayer } from "@formant/universe";
import * as THREE from "three";

export class LightsaberLayer extends UniverseLayer {
  static layerTypeId = "lightsaber";
  static commonName = "Lightsaber";
  static description = "A weapon of a more civilized age.";

  geo = new THREE.BoxGeometry(0.04318, 0.04318, 0.91);
  mat = new THREE.MeshBasicMaterial({ color: 0x20a0ff });
  cube = new THREE.Mesh(this.geo, this.mat);

  init() {
    this.add(this.cube);
  }

  onEnterVR(xr: THREE.WebXRManager): void {
    xr.getControllerGrip(0).add(this.cube);
  }

  onExitVR(xr: THREE.WebXRManager): void {
    xr.getControllerGrip(0).remove(this.cube);
  }

  destroy(): void {
    this.geo.dispose();
    this.mat.dispose();
  }
}
