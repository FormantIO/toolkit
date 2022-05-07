import { UniverseLayer, FormantHandModel, Label } from "@formant/universe";
import * as THREE from "three";

export class CubeLayer extends UniverseLayer {
  static layerTypeId = "cube";
  static commonName = "Cube";
  static description = "This is just a simple cube.";

  geo = new THREE.BoxGeometry(1, 1, 1);
  mat = new THREE.MeshBasicMaterial({ color: 0x20a0ff });
  cube = new THREE.Mesh(this.geo, this.mat);

  label = new Label("hey");

  init() {
    this.add(this.cube);
    this.add(this.label);
  }

  onHandsMoved(hands: FormantHandModel[]): void {
    this.label.text = JSON.stringify((hands[0].controller as any).joints);
  }

  destroy(): void {
    this.geo.dispose();
    this.mat.dispose();
  }
}
