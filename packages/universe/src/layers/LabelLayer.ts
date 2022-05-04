import { Label } from "../objects/Label";
import { IUniverseData, UniverseDataSource } from "../model/IUniverseData";
import { TransformLayer } from "./TransformLayer";
import { UniverseLayerContent } from "./UniverseLayerContent";
import { LayerFields, LayerField } from "../model/LayerField";
import { defined } from "../../../common/defined";

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
    layerId: string,
    _universeData: IUniverseData,
    deviceId: string,
    _universeDataSources?: UniverseDataSource[],
    fields?: LayerFields
  ): TransformLayer<LabelLayer> {
    return new TransformLayer(
      layerId,
      new LabelLayer(layerId, (fields || {}).label_text),
      deviceId
    );
  }

  label: Label | undefined;

  constructor(layerId?: string, labelTextField?: LayerField) {
    super(defined(layerId));
    if (
      labelTextField &&
      labelTextField.type === "text" &&
      labelTextField.value
    ) {
      this.createLabel(labelTextField.value);
    }
  }

  createLabel(text: string): void {
    if (this.label) {
      this.remove(this.label);
      this.label.dispose();
    }
    this.label = new Label(text);
    this.add(this.label);
  }

  onFieldChanged(_field: string, value: string): void {
    this.createLabel(value);
  }
}
