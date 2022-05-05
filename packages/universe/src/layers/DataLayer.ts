import { UniverseLayer } from "./UniverseLayer";

export class DataLayer extends UniverseLayer {
  static layerTypeId = "data";

  static commonName = "Data Source";

  static description = "A layer that connects data to layers beneath";

  static usesData = false;
}
