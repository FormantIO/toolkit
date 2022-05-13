import { GroundPlane } from "../objects/GroundPlane";
import { UniverseLayer } from "./UniverseLayer";

export class GroundLayer extends UniverseLayer {
  static layerTypeId: string = "ground";

  static commonName = "Ground";

  static description = "A flat plane to represent the ground.";

  static usesData = false;

  init() {
    this.add(new GroundPlane());
  }
}
