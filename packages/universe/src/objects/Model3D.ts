import { Object3D } from "three";
import { GLTFLoader } from "../../three-utils/loaders/GLTFLoader";

export class Model3D extends Object3D {
  constructor(url: string) {
    super();
    const loader = new GLTFLoader();
    loader.load(url, (gltf) => {
      this.add(gltf.scene);
    });
  }
}
