import { Object3D, PerspectiveCamera, Raycaster } from "three";
import { getRecoil, setRecoil } from "recoil-nexus";
import { IUniverseData, UniverseDataSource } from "../model/IUniverseData";
import { LayerSuggestion } from "./LayerRegistry";
import { TransformLayer } from "./TransformLayer";
import { LayerFields } from "../model/LayerField";
import { snackbarAtom } from "../state/snackbar";
import { sceneGraphAtom } from "../state/sceneGraph";
import { SceneGraph, visitSceneGraphElement } from "../main";

export abstract class UniverseLayerContent extends Object3D {
  static id: string;

  static commonName: string;

  static description: string;

  static usesData: boolean;

  static fields?: LayerFields;

  layerId: string;

  static async getLayerSuggestions(
    _data: IUniverseData,
    _deviceContext?: string
  ): Promise<LayerSuggestion[]> {
    return [];
  }

  static createDefault(
    _layerId: string,
    _universeData: IUniverseData,
    _deviceId?: string,
    _universeDataSources?: UniverseDataSource[],
    _fields?: LayerFields,
    _camera?: () => PerspectiveCamera
  ): TransformLayer<UniverseLayerContent> {
    throw new Error("Method not implemented.");
  }

  constructor(layerId: string) {
    super();
    this.layerId = layerId;
  }

  onPointerMove(_raycaster: Raycaster): void {}

  onPointerEnter(_raycaster: Raycaster): void {}

  onPointerLeave(_raycaster: Raycaster): void {}

  onPointerDown(_raycaster: Raycaster, _button: number): void {}

  onPointerUp(_raycaster: Raycaster, _button: number): void {}

  onPointerWheel(_raycaster: Raycaster, _delta: number): void {}

  onFieldChanged(_field: string, _value: string): void {}

  showSnackbar(message: string): void {
    setRecoil(snackbarAtom, { message, open: true });
  }

  setFieldValue(field: string, value: string): void {
    // TODO: do some sort of deep nested immutable recreation
    const sceneGraph = JSON.parse(
      JSON.stringify(getRecoil(sceneGraphAtom))
    ) as SceneGraph;
    sceneGraph.forEach((element) => {
      visitSceneGraphElement(element, (e) => {
        if (e.id === this.layerId) {
          e.fieldValues[field].value = value;
        }
      });
    });
    setRecoil(sceneGraphAtom, sceneGraph);
  }
}
