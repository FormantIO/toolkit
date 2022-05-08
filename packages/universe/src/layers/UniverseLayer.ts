import { Object3D, PerspectiveCamera, Raycaster, WebXRManager } from "three";
import { getRecoil, setRecoil } from "recoil-nexus";
import { IUniverseData, UniverseDataSource } from "../model/IUniverseData";
import { LayerSuggestion } from "./LayerRegistry";
import { TransformLayer } from "./TransformLayer";
import { LayerFields } from "../model/LayerField";
import { snackbarAtom } from "../state/snackbar";
import { sceneGraphAtom } from "../state/sceneGraph";
import { SceneGraph, visitSceneGraphElement } from "../main";
import { Hand } from "../components/viewer/Hand";
import { Controller } from "../components/viewer/Controller";

export abstract class UniverseLayer extends Object3D {
  static layerTypeId: string;

  static commonName: string;

  static description: string;

  static usesData: boolean;

  static fields?: LayerFields;

  protected layerId!: string;

  protected universeData!: IUniverseData;

  protected layerContext?: string;

  protected layerDataSources: UniverseDataSource[] = [];

  protected layerFields: LayerFields = {};

  camera?: () => PerspectiveCamera;

  static async getLayerSuggestions(
    _data: IUniverseData,
    _deviceContext?: string
  ): Promise<LayerSuggestion[]> {
    return [];
  }

  static createDefault<T extends UniverseLayer>(
    content: T,
    layerId: string,
    universeData: IUniverseData,
    deviceId?: string,
    dataSources?: UniverseDataSource[],
    fields?: LayerFields,
    camera?: () => PerspectiveCamera
  ): TransformLayer {
    content.layerId = layerId;
    content.universeData = universeData;
    content.layerContext = deviceId;
    content.layerDataSources = dataSources || [];
    content.layerFields = fields || {};
    content.camera = camera;
    const transform = new TransformLayer();
    transform.universeData = universeData;
    transform.deviceId = deviceId;
    transform.contentNode = content;
    transform.add(content);
    content.init();
    return transform;
  }

  init(): void {}

  onPointerMove(_raycaster: Raycaster): void {}

  onPointerEnter(_raycaster: Raycaster): void {}

  onPointerLeave(_raycaster: Raycaster): void {}

  onPointerDown(_raycaster: Raycaster, _button: number): void {}

  onPointerUp(_raycaster: Raycaster, _button: number): void {}

  onPointerWheel(_raycaster: Raycaster, _delta: number): void {}

  onFieldChanged(_field: string, _value: string): void {}

  onEnterVR(_xr: WebXRManager): void {}

  onExitVR(_xr: WebXRManager): void {}

  onVisibilityChanged(_visibility: boolean): void {}

  onControllerButtonChanged(
    _controller: Controller,
    _raycaster: Raycaster,
    _button: number,
    _value: number
  ): void {}

  onControllerAxisChanged(
    _controller: Controller,
    _raycaster: Raycaster,
    _axis: number,
    _value: number
  ): void {}

  onControllersMoved(
    _controllers: Controller[],
    _raycasters: Raycaster[]
  ): void {}

  onHandsEnter(_hands: Hand[]): void {}

  onHandsLeave(_hands: Hand[]): void {}

  onHandsMoved(_hands: Hand[]): void {}

  onUpdate(_delta: number): void {}

  destroy(): void {}

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
