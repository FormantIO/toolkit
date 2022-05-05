import {
  DoubleSide,
  Group,
  Mesh,
  MeshBasicMaterial,
  PlaneBufferGeometry,
  ShapeGeometry,
  Texture,
  TextureLoader,
} from "three";
import { UniverseLayer } from "./UniverseLayer";
import { SVGLoader } from "../../three-utils/SVGLoader";

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
      const isSvg = urlField.value.toLowerCase().endsWith(".svg");
      if (isSvg) {
        const loader = new SVGLoader();
        loader.load(urlField.value, (data) => {
          const { paths } = data;
          const group = new Group();

          paths.forEach((path) => {
            const material = new MeshBasicMaterial({
              color: path.color,
              side: DoubleSide,
              depthWrite: false,
            });

            const shapes = path.toShapes(false);

            shapes.forEach((shape) => {
              const geometry = new ShapeGeometry(shape);
              const mesh = new Mesh(geometry, material);
              group.add(mesh);
            });
          });
          const oneEightyDegrees = Math.PI;
          group.rotation.set(oneEightyDegrees, 0, 0);
          group.scale.set(0.001, 0.001, 0.001);
          this.add(group);
        });
      } else {
        new TextureLoader().load(urlField.value, (texture: Texture) => {
          let w = texture.image.width;
          let h = texture.image.height;
          if (w > h) {
            h /= w;
            w = 1;
          } else {
            w /= h;
            h = 1;
          }
          const geometry = new PlaneBufferGeometry(w, h);
          const material = new MeshBasicMaterial({
            map: texture,
            transparent: true,
          });
          const mesh = new Mesh(geometry, material);
          this.add(mesh);
        });
      }
    }
  }
}
