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
      placeholder: "false",
      value: "false",
      type: "text",
      location: ["create"],
    },
  };

  init() {
    const { flatAxes } = this.layerFields || {};
    const flat =
      flatAxes && flatAxes.type === "text" && flatAxes.value === "true";

    this.add(new GroundPlane());
    this.add(new Axes(flat));
    this.add(new AxisLabels(flat));
  }
}
