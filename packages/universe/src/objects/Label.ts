import { Group, Sprite, SpriteMaterial, Texture } from "three";
import { definedAndNotNull } from "../../../common/defined";

function roundRect(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  width: number,
  height: number,
  radius: number,
  fill: string
) {
  ctx.fillStyle = fill;
  ctx.beginPath();
  ctx.moveTo(x + radius, y);
  ctx.lineTo(x + width - radius, y);
  ctx.quadraticCurveTo(x + width, y, x + width, y + radius);
  ctx.lineTo(x + width, y + height - radius);
  ctx.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
  ctx.lineTo(x + radius, y + height);
  ctx.quadraticCurveTo(x, y + height, x, y + height - radius);
  ctx.lineTo(x, y + radius);
  ctx.quadraticCurveTo(x, y, x + radius, y);
  ctx.closePath();
  ctx.fill();
}

export class Label extends Group {
  sprite: Sprite | undefined;

  texture: Texture | undefined;

  material: SpriteMaterial | undefined;

  currentText: string;

  constructor(text: string, private sizeAttenuate: boolean = true) {
    super();
    this.currentText = text;
    this.update();
  }

  private update() {
    if (this.sprite) {
      this.dispose();
      this.sprite = undefined;
    }
    const fontface = "Arial";
    const fontsize = 30;
    const message = this.currentText;
    const font = `${fontsize}px ${fontface}`;

    const canvas = document.createElement("canvas");
    const context = definedAndNotNull(canvas.getContext("2d"));

    // get size data (height depends only on font size)
    context.font = font;
    const metrics = context.measureText(message);
    const textWidth = metrics.width;
    const textHeight = fontsize * 1.5;
    const padding = 20;
    canvas.width = textWidth + padding;
    canvas.height = textHeight + padding;
    context.globalAlpha = 0.5;
    roundRect(
      context,
      0,
      0,
      textWidth + padding,
      textHeight + padding,
      10,
      "#2d3855"
    );
    context.globalAlpha = 1;

    // background color
    context.font = font;
    context.fillStyle = "#bac4e2";
    context.fillText(message, 0 + 10, fontsize + 10);

    // canvas contents will be used for a texture
    const texture = new Texture(canvas);
    this.texture = texture;
    texture.needsUpdate = true;

    const spriteMaterial = new SpriteMaterial({
      map: texture,
      depthTest: false,
      sizeAttenuation: this.sizeAttenuate,
    });
    this.material = spriteMaterial;

    const sprite = new Sprite(spriteMaterial);
    // make things less blurrier
    const pixelScale = (this.sizeAttenuate ? 0.002 : 0.001) * canvas.width;
    // scale sprite so it isn't stretched
    sprite.scale.set(
      pixelScale,
      ((textHeight + padding) / (textWidth + padding)) * pixelScale,
      pixelScale
    );
    this.add(sprite);
    this.sprite = sprite;
    this.renderOrder = 100;
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
