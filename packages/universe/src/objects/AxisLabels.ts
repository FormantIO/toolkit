import { Group } from "three";
import { Label } from "./Label";

export class AxisLabels extends Group {
  private labelRadius: number = 1;

  constructor(flat: boolean) {
    super();
    const x = new Label("x", false);
    x.position.set(this.labelRadius, 0, 0);
    this.add(x);
    const y = new Label("y", false);
    y.position.set(0, this.labelRadius, 0);
    this.add(y);
    if (!flat) {
      const z = new Label("z", false);
      z.position.set(0, 0, this.labelRadius);
      this.add(z);
    }
  }
}
