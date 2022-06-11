/* eslint-disable no-bitwise */
import {
  DoubleSide,
  Group,
  LinearMipMapLinearFilter,
  Mesh,
  MeshBasicMaterial,
  PlaneBufferGeometry,
  Texture,
} from "three";
import { definedAndNotNull } from "../../../common/defined";

export class TextPlane extends Group {
  mesh: Mesh | undefined;

  texture: Texture | undefined;

  material: MeshBasicMaterial | undefined;

  currentText: string;

  currentColor: number;

  currentFontSize: number;

  currentMaxWidth?: number;

  constructor(
    text: string,
    color: number = 0xbac4e2,
    fontSize: number = 30,
    maxWidth: number | undefined = undefined
  ) {
    super();
    this.currentText = text;
    this.currentColor = color;
    this.currentFontSize = fontSize;
    this.currentMaxWidth = maxWidth;
    this.update();
  }

  private toColor(num: number) {
    let n = num;
    n >>>= 0;
    const b = n & 0xff;
    const g = (n & 0xff00) >>> 8;
    const r = (n & 0xff0000) >>> 16;
    const c = `rgba(${r},${g},${b},1)`;
    return c;
  }

  private update() {
    if (this.mesh) {
      this.dispose();
      this.remove(this.mesh);
      this.mesh = undefined;
    }
    const fontface = "Inter";
    const fontsize = this.currentFontSize;
    const message = this.currentText;
    const font = `${fontsize * 4}px ${fontface}`;

    const canvas = document.createElement("canvas");
    const context = definedAndNotNull(canvas.getContext("2d"));

    // get size data (height depends only on font size)
    context.font = font;
    const metrics = context.measureText(message);
    const actualHeight =
      metrics.fontBoundingBoxAscent + metrics.fontBoundingBoxDescent;
    const textWidth = metrics.width;
    const textHeight = actualHeight;
    canvas.width = textWidth;
    canvas.height = textHeight;

    // background color
    context.font = font;
    context.textBaseline = "top";
    context.fillStyle = this.toColor(this.currentColor);
    context.fillText(message, 0, 0);

    // canvas contents will be used for a texture
    const texture = new Texture(canvas);
    this.texture = texture;
    texture.needsUpdate = true;
    texture.minFilter = LinearMipMapLinearFilter;
    texture.magFilter = LinearMipMapLinearFilter;

    const material = new MeshBasicMaterial({
      map: texture,
      transparent: true,
      side: DoubleSide,
    });

    this.material = material;

    const mesh = new Mesh(
      new PlaneBufferGeometry(textWidth, textHeight),
      material
    );

    // make things less blurrier
    const pixelScale = 0.000005 * canvas.width;
    mesh.scale.set(pixelScale, pixelScale, pixelScale);
    this.add(mesh);
    this.mesh = mesh;
  }

  get text(): string {
    return this.currentText;
  }

  set text(v: string) {
    if (this.currentText !== v) {
      this.currentText = v;
      this.update();
    }
  }

  get color(): number {
    return this.currentColor;
  }

  set color(v: number) {
    if (this.currentColor !== v) {
      this.currentColor = v;
      this.update();
    }
  }

  get fontSize(): number {
    return this.currentFontSize;
  }

  set fontSize(v: number) {
    if (this.currentFontSize !== v) {
      this.currentFontSize = v;
      this.update();
    }
  }

  get maxWidth(): number | undefined {
    return this.currentMaxWidth;
  }

  set maxWidth(v: number | undefined) {
    if (this.currentMaxWidth !== v) {
      this.currentMaxWidth = v;
      this.update();
    }
  }

  dispose() {
    if (this.texture) {
      this.texture.dispose();
      this.texture = undefined;
    }
    if (this.material) {
      this.material.dispose();
      this.material = undefined;
    }
  }
}
