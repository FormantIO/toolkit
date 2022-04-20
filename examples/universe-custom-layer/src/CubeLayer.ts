import {
  IUniverseData,
  TransformLayer,
  UniverseDataSource,
  UniverseLayerContent,
} from "@formant/universe";
import * as THREE from "three";

export class CubeLayer extends UniverseLayerContent {
  static id = "cube";
  static commonName = "Cube";
  static description = "A cube.";
  static usesData = false;

  static createDefault(
    _universeData: IUniverseData,
    _deviceId: string,
    _universeDataSources?: UniverseDataSource[]
  ): TransformLayer<CubeLayer> {
    return new TransformLayer(new CubeLayer());
  }

  constructor() {
    super();
    const cube = new THREE.Mesh(
      new THREE.BoxGeometry(1, 1, 1),
      new THREE.MeshBasicMaterial({ color: 0x20a0ff })
    );
    this.add(cube);
  }
}
