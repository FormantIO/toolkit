import { UniverseLayer } from "@formant/universe";
import * as THREE from "three";

export class CubeLayer extends UniverseLayer {
  static layerTypeId = "cube";
  static commonName = "Cube";
  static description = "This is just a simple cube.";

  geo = new THREE.BoxGeometry(1, 1, 1);
  mat = new THREE.MeshBasicMaterial({ color: 0x20a0ff });
  cube = new THREE.Mesh(this.geo, this.mat);

  init() {
    this.add(this.cube);
  }

  onPointerMove(raycaster: THREE.Raycaster): void {
    let intersects = raycaster.intersectObject(this.cube).length > 0;
    this.mat.color.set(intersects ? 0x20a0ff : 0xffffff);
  }

  onPointerDown(raycaster: THREE.Raycaster): void {
    if (raycaster.intersectObject(this.cube).length > 0) {
      this.mat.color.set(0xff0000);
    }
  }

  onPointerUp(_raycaster: THREE.Raycaster, _button: number): void {
    this.showSnackbar("Clicked!");
  }

  onEnterVR(_xr: THREE.WebXRManager): void {
    this.mat.color.set(0x00ff00);
  }

  onExitVR(_xr: THREE.WebXRManager): void {
    this.mat.color.set(0xffffff);
  }

  destroy(): void {
    this.geo.dispose();
    this.mat.dispose();
  }
}
