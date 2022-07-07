import { GroundPlane } from "../objects/GroundPlane";
import { Axes } from "../objects/Axes";
import { AxisLabels } from "../objects/AxisLabels";
import { UniverseLayer } from "./UniverseLayer";

export class GroundLayer extends UniverseLayer {
  static layerTypeId: string = "ground";

  static commonName = "Ground";

  static description = "A flat plane to represent the ground.";

  static usesData = false;

  static fields = {
    flatAxes: {
      name: "Show Flat Axes",
      description: "Axes should be flat",
      placeholder: false,
      value: false,
      type: "boolean" as const,
      location: ["create" as const],
    },
  };

  init() {
    const flat = this.getField(GroundLayer.fields.flatAxes) || false;

    this.add(new GroundPlane());
    this.add(new Axes(flat));
    this.add(new AxisLabels(flat));
  }
}
