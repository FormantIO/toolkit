import { Axes } from "../objects/Axes";
import { AxisLabels } from "../objects/AxisLabels";
import { UniverseLayer } from "./UniverseLayer";

export class AxesLayer extends UniverseLayer {
  static layerTypeId: string = "axes";

  static commonName = "Axes";

  static description = "Axes for the ground that show cardinal directions";

  static usesData = false;

  init() {
    this.add(new Axes());
    const axes = new AxisLabels();
    this.add(axes);
  }
}
