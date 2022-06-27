import { Object3D } from "three";
import { GLTFLoader } from "../../three-utils/loaders/GLTFLoader";

export class Model3D extends Object3D {
  constructor(url: string, onLoad?: () => void) {
    super();
    this.rotateX(Math.PI / 2);
    const loader = new GLTFLoader();
    loader.load(url, (gltf) => {
      this.add(gltf.scene);
      if (onLoad) onLoad();
    });
  }
}
