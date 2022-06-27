import { UniverseLayer } from "./UniverseLayer";

export class EmptyLayer extends UniverseLayer {
  static layerTypeId: string = "empty";

  static commonName = "Empty Layer";

  static description = "Empty layer";

  static usesData = false;
}
