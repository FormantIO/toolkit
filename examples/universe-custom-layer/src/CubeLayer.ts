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

  cube = new THREE.Mesh(
    new THREE.BoxGeometry(1, 1, 1),
    new THREE.MeshBasicMaterial({ color: 0x20a0ff })
  );

  static createDefault(
    layerId: string,
    _universeData: IUniverseData,
    _deviceId: string,
    _universeDataSources?: UniverseDataSource[]
  ): TransformLayer<CubeLayer> {
    return new TransformLayer(layerId, new CubeLayer(layerId));
  }

  constructor(layerId?: string) {
    super(layerId as string);
    this.add(this.cube);
  }

  onPointerMove(raycaster: THREE.Raycaster): void {
    let intersects = false;
    if (raycaster.intersectObject(this.cube).length > 0) {
      intersects = true;
    }
    let m;
    if (Array.isArray(this.cube.material)) {
      m = this.cube.material[0];
    } else {
      m = this.cube.material;
    }
    if (m instanceof THREE.MeshBasicMaterial) {
      m.color.set(intersects ? 0x20a0ff : 0xffffff);
    }
  }

  onPointerDown(raycaster: THREE.Raycaster): void {
    if (raycaster.intersectObject(this.cube).length > 0) {
      let m;
      if (Array.isArray(this.cube.material)) {
        m = this.cube.material[0];
      } else {
        m = this.cube.material;
      }
      if (m instanceof THREE.MeshBasicMaterial) {
        m.color.set(0xff0000);
      }
    }
  }

  onPointerUp(_raycaster: THREE.Raycaster, _button: number): void {
    this.showSnackbar("Clicked!");
  }
}
