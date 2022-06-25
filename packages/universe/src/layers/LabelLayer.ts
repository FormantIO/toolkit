import { defined } from "../main";
import { Label } from "../objects/Label";
import { UniverseLayer } from "./UniverseLayer";

export class LabelLayer extends UniverseLayer {
  static layerTypeId: string = "label";

  static commonName = "Label";

  static description = "A text label";

  static usesData = false;

  static fields = {
    labelText: {
      name: "Label Text",
      description: "The text you'd like to show in the label",
      placeholder: "hello world",
      value: "",
      type: "text" as const,
      location: ["create" as const, "edit" as const],
    },
  };

  label: Label | undefined;

  init() {
    this.updateLabel(defined(this.getField(LabelLayer.fields.labelText)));
  }

  updateLabel(text: string): void {
    if (this.label) {
      this.remove(this.label);
      this.label.dispose();
    }
    this.label = new Label(text);
    this.add(this.label);
  }

  onFieldChanged(_field: string, value: string): void {
    this.updateLabel(value);
  }
}
