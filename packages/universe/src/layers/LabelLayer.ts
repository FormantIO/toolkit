import { Sprite, SpriteMaterial, Texture } from "three";
import { definedAndNotNull } from "../../../common/defined";
import { IUniverseData, UniverseDataSource } from "../IUniverseData";
import { TransformLayer } from "./TransformLayer";
import {
  LayerField,
  LayerFields,
  UniverseLayerContent,
} from "./UniverseLayerContent";

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
export class LabelLayer extends UniverseLayerContent {
  static id = "label";
  static commonName = "Label";
  static description = "A text label";
  static usesData = false;
  static fields = {
    label_text: {
      name: "Label Text",
      description: "The text you'd like to show in the label",
      placeholder: "hello world",
      value: "",
      type: "text",
    },
  };

  static createDefault(
    _universeData: IUniverseData,
    _deviceId: string,
    _universeDataSources?: UniverseDataSource[],
    fields?: LayerFields
  ): TransformLayer<LabelLayer> {
    return new TransformLayer(new LabelLayer((fields || {}).label_text));
  }

  constructor(labelTextField?: LayerField) {
    super();
    if (
      labelTextField &&
      labelTextField.type === "text" &&
      labelTextField.value
    ) {
      const fontface = "Arial";
      const fontsize = 30;
      const message = labelTextField.value;
      const font = fontsize + "px " + fontface;

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
      context.fillStyle = "#000000";
      roundRect(
        context,
        0,
        0,
        textWidth + padding,
        textHeight + padding,
        10,
        "#000000"
      );

      // background color
      context.font = font;
      context.fillStyle = "#bac4e2";
      context.fillText(message, 0 + 10, fontsize + 10);

      // canvas contents will be used for a texture
      const texture = new Texture(canvas);
      texture.needsUpdate = true;

      const spriteMaterial = new SpriteMaterial({
        map: texture,
        depthTest: false,
      });

      const sprite = new Sprite(spriteMaterial);
      // make things less blurrier
      const pixelScale = 4;
      // scale sprite so it isn't stretched
      sprite.scale.set(
        1 / pixelScale,
        (textHeight + padding) / (textWidth + padding) / pixelScale,
        1.0 / pixelScale
      );
      this.add(sprite);
      this.renderOrder = 100;
    }
  }
}
