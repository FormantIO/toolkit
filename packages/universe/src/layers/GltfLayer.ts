import { Model3D } from "../objects/Model3D";
import { UniverseLayer } from "./UniverseLayer";

export class GltfLayer extends UniverseLayer {
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
      this.add(new Model3D(urlField.value));
    }
  }
}
