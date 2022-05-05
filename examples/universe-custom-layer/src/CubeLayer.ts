import { UniverseLayer } from "@formant/universe";
import * as THREE from "three";

export class CubeLayer extends UniverseLayer {
  static layerTypeId = "cube";
  static commonName = "Cube";
  static description = "This is just a simple cube.";

  cube = new THREE.Mesh(
    new THREE.BoxGeometry(1, 1, 1),
    new THREE.MeshBasicMaterial({ color: 0x20a0ff })
  );

  init() {
    this.add(this.cube);
  }

  onPointerMove(raycaster: THREE.Raycaster): void {
    let intersects = raycaster.intersectObject(this.cube).length > 0;
    (this.cube.material as THREE.MeshBasicMaterial).color.set(
      intersects ? 0x20a0ff : 0xffffff
    );
  }

  onPointerDown(raycaster: THREE.Raycaster): void {
    if (raycaster.intersectObject(this.cube).length > 0) {
      (this.cube.material as THREE.MeshBasicMaterial).color.set(0xff0000);
    }
  }

  onPointerUp(_raycaster: THREE.Raycaster, _button: number): void {
    this.showSnackbar("Clicked!");
  }
}
