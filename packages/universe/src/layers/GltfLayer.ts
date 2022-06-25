import { defined } from "../main";
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
      type: "text" as const,
      location: ["create" as const],
    },
  };

  init() {
    this.add(this.createGltf(defined(this.getField(GltfLayer.fields.url))));
  }
}
