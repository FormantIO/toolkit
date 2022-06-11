/* eslint-disable no-bitwise */
import {
  DoubleSide,
  Group,
  Mesh,
  MeshBasicMaterial,
  Object3D,
  PlaneBufferGeometry,
  ShapeGeometry,
  Texture,
  TextureLoader,
} from "three";
import { SVGLoader } from "../../three-utils/loaders/SVGLoader";

export class ImagePlane extends Object3D {
  constructor(url: string) {
    super();
    const isSvg = url.toLowerCase().endsWith(".svg");
    if (isSvg) {
      const loader = new SVGLoader();
      loader.load(url, (data) => {
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
      new TextureLoader().load(url, (texture: Texture) => {
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
          side: DoubleSide,
        });
        const mesh = new Mesh(geometry, material);
        this.add(mesh);
      });
    }
  }
}
