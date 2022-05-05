import { Label } from "../objects/Label";
import { UniverseLayerContent } from "./UniverseLayerContent";

export class LabelLayer extends UniverseLayerContent {
  static layerTypeId: string = "label";

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

  label: Label | undefined;

  init() {
    const labelTextField = (this.layerFields || {}).label_text;
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
