import { Object3D } from "three";
import { GLTFLoader } from "../../three-utils/loaders/GLTFLoader";

export class Model3D extends Object3D {
  constructor(url: string) {
    super();
    const loader = new GLTFLoader();
    loader.load(url, (gltf) => {
      const ninetyDegrees = Math.PI / 2;
      gltf.scene.rotation.set(ninetyDegrees, 0, 0);
      this.add(gltf.scene);
    });
  }
}
