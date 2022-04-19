import { Sprite, SpriteMaterial, Texture } from "three";
import { definedAndNotNull } from "../../../common/defined";
import { IUniverseData, UniverseDataSource } from "../IUniverseData";
import { TransformLayer } from "./TransformLayer";
import {
  LayerField,
  LayerFields,
  UniverseLayerContent,
} from "./UniverseLayerContent";
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
      canvas.width = textWidth;
      canvas.height = textHeight;
      context.fillStyle = "#2d3855";
      context.fillRect(0, 0, textWidth, textHeight);

      // background color
      context.font = font;
      context.fillStyle = "#bac4e2";
      context.fillText(message, 0, fontsize);

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
        textHeight / textWidth / pixelScale,
        1.0 / pixelScale
      );
      this.add(sprite);
      this.renderOrder = 100;
    }
  }
}
