import {
  Group,
  Mesh,
  MeshNormalMaterial,
  PerspectiveCamera,
  SphereGeometry,
  Vector2,
  Vector3,
} from "three";
import { measurePixel } from "./measurePixel";

export class Joint extends Group {
  public resolution: Vector2 = new Vector2();

  private sphere: Mesh;

  constructor(public camera: PerspectiveCamera) {
    super();
    const material = new MeshNormalMaterial();
    this.sphere = new Mesh(new SphereGeometry(0.5, 32, 16), material);
    this.add(this.sphere);
    this.matrixAutoUpdate = false;
  }

  public update() {
    const { camera, resolution } = this;

    const position = new Vector3().setFromMatrixPosition(
      this.sphere.matrixWorld
    );
    const scale = 10 * measurePixel(position, camera, resolution);
    this.sphere.scale.set(scale, scale, scale);
  }
}
