import { UniverseLayerContent } from "./UniverseLayerContent";

export class DataLayer extends UniverseLayerContent {
  static layerTypeId = "data";

  static commonName = "Data Source";

  static description = "A layer that connects data to layers beneath";

  static usesData = false;
}
