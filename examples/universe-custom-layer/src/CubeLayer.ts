import { UniverseLayer, FormantHandModel } from "@formant/universe";
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

  onHandsMoved(hands: FormantHandModel[]): void {
    let intersects = false;
    hands.forEach((hand) => {
      if (hand.intersectBoxObject(this.cube)) {
        intersects = true;
      }
    });

    if (intersects) {
      this.mat.color.set(0xff0000);
    } else {
      this.mat.color.set(0x20a0ff);
    }
  }

  destroy(): void {
    this.geo.dispose();
    this.mat.dispose();
  }
}
