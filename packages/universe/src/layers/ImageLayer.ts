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
      type: "text",
      location: ["create"],
    },
  };

  init() {
    const urlField = (this.layerFields || {}).url;
    if (urlField && urlField.type === "text" && urlField.value) {
      this.add(this.createImage(urlField.value));
    }
  }
}
