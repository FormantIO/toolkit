import { GroundPlane } from "../objects/GroundPlane";
import { Axes } from "../objects/Axes";
import { AxisLabels } from "../objects/AxisLabels";
import { UniverseLayer } from "./UniverseLayer";

export class GroundLayer extends UniverseLayer {
  static layerTypeId: string = "ground";

  static commonName = "Ground";

  static description = "A flat plane to represent the ground.";

  static usesData = false;

  init() {
    this.add(new GroundPlane());
    this.add(new Axes());
    const axes = new AxisLabels();
    this.add(axes);
  }
}
