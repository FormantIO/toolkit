import { defined } from "../main";
import { UniverseLayer } from "./UniverseLayer";

export class ImageLayer extends UniverseLayer {
  static layerTypeId: string = "image";

  static commonName = "Image";

  static description = "A image plane model.";

  static usesData = false;

  static fields = {
    url: {
      name: "URL",
      description: "The URL image to use as a texture",
      placeholder: "https://example.com/image.png",
      value: "",
      type: "text" as const,
      location: ["create" as const],
    },
  };

  init() {
    this.add(this.createImage(defined(this.getField(ImageLayer.fields.url))));
  }
}
