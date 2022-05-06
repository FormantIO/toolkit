import { UniverseLayer } from "@formant/universe";
import * as THREE from "three";

export class LightsaberLayer extends UniverseLayer {
  static layerTypeId = "lightsaber";
  static commonName = "Lightsaber";
  static description = "A weapon of a more civilized age.";

  saberGeo = new THREE.BoxGeometry(0.0418, 0.0418, 0.91);
  saberMat = new THREE.MeshBasicMaterial({ color: 0x18d2ff });
  saberCube = new THREE.Mesh(this.saberGeo, this.saberMat);

  baseGeo = new THREE.BoxGeometry(0.04318, 0.04318, 0.2286);
  baseMat = new THREE.MeshBasicMaterial({ color: 0xbac4e2 });
  baseCube = new THREE.Mesh(this.baseGeo, this.baseMat);

  init() {
    this.baseCube.add(this.saberCube);
    this.saberCube.position.set(0, 0, 0.5);
    this.add(this.baseCube);
  }

  onEnterVR(xr: THREE.WebXRManager): void {
    xr.getControllerGrip(0).add(this.baseCube);
  }

  onExitVR(xr: THREE.WebXRManager): void {
    xr.getControllerGrip(0).remove(this.baseCube);
  }

  destroy(): void {
    this.baseGeo.dispose();
    this.baseMat.dispose();
    this.saberGeo.dispose();
    this.saberMat.dispose();
  }
}
