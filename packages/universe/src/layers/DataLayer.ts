import { IUniverseData, UniverseDataSource } from "../model/IUniverseData";
import { TransformLayer } from "./TransformLayer";
import { UniverseLayerContent } from "./UniverseLayerContent";

export class DataLayer extends UniverseLayerContent {
  static id = "data";

  static commonName = "Data Source";

  static description = "A layer that connects data to layers beneath";

  static usesData = false;

  static createDefault(
    _universeData: IUniverseData,
    deviceId: string,
    _universeDataSources?: UniverseDataSource[]
  ): TransformLayer<DataLayer> {
    return new TransformLayer(new DataLayer(), deviceId);
  }
}
