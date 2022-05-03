import { Group } from "three";
import { Label } from "./Label";

export class AxisLabels extends Group {
  private labelRadius: number = 1;

  constructor() {
    super();
    const x = new Label("x", false, 10);
    x.position.set(this.labelRadius, 0, 0);
    this.add(x);
    const y = new Label("y", false, 10);
    y.position.set(0, this.labelRadius, 0);
    this.add(y);
    const z = new Label("z", false, 10);
    z.position.set(0, 0, this.labelRadius);
    this.add(z);
  }
}
