import {
  ConeGeometry,
  Group,
  Mesh,
  MeshNormalMaterial,
  PerspectiveCamera,
  Vector2,
  Vector3,
} from "three";
import { measurePixel } from "./measurePixel";

export class Pointer extends Group {
  public resolution: Vector2 = new Vector2();

  private pointer: Group;

  constructor(private camera: PerspectiveCamera) {
    super();
    const material = new MeshNormalMaterial();
    this.pointer = new Group();
    const cone = new Mesh(new ConeGeometry(0.5, 2, 32, 16), material);
    cone.position.set(1, 0, 0);
    cone.rotation.set(0, 0, Math.PI / 2);
    this.pointer.add(cone);
    this.add(this.pointer);
    this.matrixAutoUpdate = false;
  }

  public update() {
    const { camera, resolution } = this;

    const position = new Vector3().setFromMatrixPosition(
      this.pointer.matrixWorld
    );
    const scale = 10 * measurePixel(position, camera, resolution);
    this.pointer.scale.set(scale, scale, scale);
  }
}
