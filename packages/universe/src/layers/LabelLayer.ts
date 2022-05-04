import { Label } from "../objects/Label";
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
      location: ["create", "edit"],
    },
  };

  static createDefault(
    _universeData: IUniverseData,
    deviceId: string,
    _universeDataSources?: UniverseDataSource[],
    fields?: LayerFields
  ): TransformLayer<LabelLayer> {
    return new TransformLayer(
      new LabelLayer((fields || {}).label_text),
      deviceId
    );
  }

  constructor(labelTextField?: LayerField) {
    super();
    if (
      labelTextField &&
      labelTextField.type === "text" &&
      labelTextField.value
    ) {
      this.add(new Label(labelTextField.value));
    }
  }
}
