import { GroundPlane } from "../objects/GroundPlane";
import { IUniverseData, UniverseDataSource } from "../IUniverseData";
import { TransformLayer } from "./TransformLayer";
import { UniverseLayerContent } from "./UniverseLayerContent";

export class GroundLayer extends UniverseLayerContent {
  static id = "ground";

  static commonName = "Ground";

  static description = "A flat plane to represent the ground.";

  static usesData = false;

  static createDefault(
    _universeData: IUniverseData,
    _deviceId: string,
    _universeDataSources?: UniverseDataSource[]
  ): TransformLayer<GroundLayer> {
    return new TransformLayer(new GroundLayer());
  }

  constructor() {
    super();
    this.add(new GroundPlane());
  }
}
