import { GLTFLoader } from "../../three-utils/GLTFLoader";
import { UniverseLayerContent } from "./UniverseLayerContent";

export class GltfLayer extends UniverseLayerContent {
  static layerTypeId: string = "3dmodel";

  static commonName = "3D Model";

  static description = "A 3D model.";

  static usesData = false;

  static fields = {
    url: {
      name: "URL",
      description: "The URL of the 3D model (*.gltf only)",
      placeholder: "https://example.com/model.gltf",
      value: "",
      type: "text",
      location: ["create"],
    },
  };

  init() {
    const urlField = (this.layerFields || {}).url;
    if (urlField && urlField.type === "text" && urlField.value) {
      const loader = new GLTFLoader();
      loader.load(urlField.value, (gltf) => {
        const ninetyDegrees = Math.PI / 2;
        gltf.scene.rotation.set(ninetyDegrees, 0, 0);
        this.add(gltf.scene);
      });
    }
  }
}
