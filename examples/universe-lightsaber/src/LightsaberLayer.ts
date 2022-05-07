import { UniverseLayer } from "@formant/universe";
import * as THREE from "three";

export class LightsaberLayer extends UniverseLayer {
  static layerTypeId = "lightsaber";
  static commonName = "Lightsaber";
  static description = "A weapon of a more civilized age.";

  saberGeo = new THREE.BoxGeometry(0.0418, 0.0418, 0.91);
  saberMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
  saberCube = new THREE.Mesh(this.saberGeo, this.saberMat);

  baseGeo = new THREE.BoxGeometry(0.04318, 0.04318, 0.2286);
  baseMat = new THREE.MeshBasicMaterial({ color: 0xbac4e2 });
  baseCube = new THREE.Mesh(this.baseGeo, this.baseMat);

  init() {
    this.baseCube.add(this.saberCube);
    this.saberCube.position.set(0, 0, -0.5);
    this.add(this.baseCube);
  }

  onEnterVR(xr: THREE.WebXRManager): void {
    xr.getControllerGrip(1).add(this.baseCube);
  }

  onExitVR(xr: THREE.WebXRManager): void {
    xr.getControllerGrip(1).remove(this.baseCube);
  }

  onGamePadButtonChanged(
    _source: THREE.XRInputSource,
    _button: number,
    value: number
  ): void {
    if (value === 1) {
      const randomColor = Math.random() * 0xffffff;
      this.saberMat.color.set(randomColor);
    }
  }

  destroy(): void {
    this.baseGeo.dispose();
    this.baseMat.dispose();
    this.saberGeo.dispose();
    this.saberMat.dispose();
  }
}
