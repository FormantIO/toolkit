import { Object3D } from "three";
import { IUniverseData, UniverseDataSource } from "../IUniverseData";
import { TransformLayer } from "./TransformLayer";
import { UniverseLayerContent } from "./UniverseLayerContent";

export class DataLayer extends UniverseLayerContent {
  static id = "data";

  static commonName = "Data Source";

  static description = "A layer that connects data to layers beneath";

  static usesData = false;

  static createDefault(
    _universeData: IUniverseData,
    _deviceId: string,
    _universeDataSources?: UniverseDataSource[],
  ): TransformLayer<Object3D> {
    return new TransformLayer(new DataLayer());
  }

  constructor() {
    super();
  }
}
