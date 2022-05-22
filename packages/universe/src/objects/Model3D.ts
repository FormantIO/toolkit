import { Color, Object3D } from "three";
import { GLTFLoader } from "../../three-utils/loaders/GLTFLoader";

export class Model3D extends Object3D {
  constructor(url: string) {
    super();
    const loader = new GLTFLoader();
    loader.load(url, (gltf) => {
      gltf.scene.traverse((o: any) => {
        if (o.isMesh) {
          if (o.material.emissive !== undefined)
            o.material.emissive = new Color(0x555555);
        }
      });
      this.add(gltf.scene);
    });
  }
}
