import {
  Group,
  Material,
  Mesh,
  Object3D,
  PerspectiveCamera,
  Raycaster,
  Vector3,
  WebXRManager,
} from "three";
import { getRecoil, setRecoil } from "recoil-nexus";
import { Howl } from "howler";
import { IUniverseData, UniverseDataSource } from "../model/IUniverseData";
import { LayerSuggestion } from "./LayerRegistry";
import { TransformLayer } from "./TransformLayer";
import { LayerFields } from "../model/LayerField";
import { snackbarAtom } from "../state/snackbar";
import { sceneGraphAtom } from "../state/sceneGraph";
import { SceneGraph, SceneGraphElement, visitSceneGraphElement } from "../main";
import { Hand } from "../components/viewer/Hand";
import { Controller } from "../components/viewer/Controller";
import { HandheldController } from "../components/viewer/HandheldController";
import { defined } from "../../../common/defined";

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

  private camera?: () => PerspectiveCamera;

  private getTransformLayerHelper?: (id: string) => TransformLayer;

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
    camera?: () => PerspectiveCamera,
    getTransformLayerHelper?: (id: string) => TransformLayer
  ): TransformLayer {
    content.layerId = layerId;
    content.universeData = universeData;
    content.layerContext = deviceId;
    content.layerDataSources = dataSources || [];
    content.layerFields = fields || {};
    content.camera = camera;
    content.getTransformLayerHelper = getTransformLayerHelper;
    const transform = new TransformLayer();
    transform.universeData = universeData;
    transform.deviceId = deviceId;
    transform.contentNode = content;
    transform.add(content);
    window.setTimeout(() => {
      content.init();
    }, 0);
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

  onVisibilityChanged(visible: boolean): void {
    // default visibility shift
    this.traverse((child) => {
      child.visible = visible;
    });
  }

  getCurrentCamera(): PerspectiveCamera {
    const f = defined(this.camera);
    return f();
  }

  getLayers(): SceneGraphElement[] {
    const sceneGraph = getRecoil(sceneGraphAtom);
    const layers: SceneGraphElement[] = [];
    sceneGraph.forEach((element) => {
      visitSceneGraphElement(element, (e) => {
        layers.push(e);
      });
    });
    return layers;
  }

  getLayerRootObject3D(layerId: string): Object3D {
    const f = defined(this.getTransformLayerHelper);
    return f(layerId);
  }

  getLayerParts(layerId: string): {
    [key in string]: Material | Mesh | Group | Object3D | undefined;
  } {
    const f = defined(this.getTransformLayerHelper);
    const tranformLayer = f(layerId);
    return defined(tranformLayer.contentNode).onLayerPartsRequested();
  }

  setLayerVisibility(id: string, visible: boolean): void {
    const sceneGraph = JSON.parse(
      JSON.stringify(getRecoil(sceneGraphAtom))
    ) as SceneGraph;
    sceneGraph.forEach((element) => {
      visitSceneGraphElement(element, (e) => {
        if (e.id === id) {
          e.visible = visible;
        }
      });
    });
    setRecoil(sceneGraphAtom, sceneGraph);
  }

  onLayerPartsRequested(): {
    [key in string]: Material | Mesh | Group | Object3D | undefined;
  } {
    return {};
  }

  onControllerButtonChanged(
    _controller: Controller,
    _button: number,
    _value: number
  ): void {}

  onControllerAxisChanged(
    _controller: Controller,
    _axis: number,
    _value: number
  ): void {}

  onHandheldControllerButtonChanged(
    _controller: HandheldController,
    _button: number,
    _value: number
  ): void {}

  onHandheldControllerAxisChanged(
    _controller: HandheldController,
    _axis: number,
    _value: number
  ): void {}

  onControllersMoved(_controllers: Controller[]): void {}

  onHandsEnter(_hands: Hand[]): void {}

  onHandsLeave(_hands: Hand[]): void {}

  onHandsMoved(_hands: Hand[]): void {}

  onHandPosesChanged(_hands: Hand[]): void {}

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

  public playSound(
    url: string,
    volume: number = 1,
    loop: boolean = false,
    pos: Vector3 | undefined = undefined
  ): () => void {
    const sound = new Howl({
      src: [url],
      volume,
      loop,
    });
    if (pos) {
      sound.pos(pos.x, pos.y, pos.z);
    }
    sound.play();
    return () => {
      sound.stop();
    };
  }
}
