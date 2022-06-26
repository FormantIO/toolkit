import {
  BufferAttribute,
  BufferGeometry,
  DoubleSide,
  Group,
  Material,
  Mesh,
  MeshBasicMaterial,
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
import {
  LayerField,
  LayerFields,
  LayerFieldType,
  LayerFieldTypeMap,
} from "../model/LayerField";
import { snackbarAtom } from "../state/snackbar";
import { sceneGraphAtom } from "../state/sceneGraph";
import {
  findSceneGraphElement,
  ImagePlane,
  Label,
  Model3D,
  SceneGraph,
  SceneGraphElement,
  TextPlane,
  visitSceneGraphElement,
} from "../main";
import { Hand } from "../components/viewer/Hand";
import { Controller } from "../components/viewer/Controller";
import { HandheldController } from "../components/viewer/HandheldController";
import { defined, definedAndNotNull } from "../../../common/defined";
import { Urdf } from "../objects/Urdf";
import { TreePath } from "../model/ITreeElement";

export type UniverseLayerContext = {
  type: "device";
  deviceId: string;
};

const typeMap = {
  text: "string",
  number: "number",
  boolean: "boolean",
};

export abstract class UniverseLayer extends Object3D {
  static layerTypeId: string;

  static commonName: string;

  static description: string;

  static usesData: boolean;

  static fields?: LayerFields;

  protected layerId!: string;

  protected universeData!: IUniverseData;

  private layerContext?: string;

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

  onFieldChanged(
    _field: string,
    _value: LayerFieldTypeMap[LayerFieldType]
  ): void {}

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

  getLayerContext(): UniverseLayerContext | undefined {
    if (this.layerContext) {
      return {
        type: "device",
        deviceId: this.layerContext,
      };
    }

    const sceneGraph = getRecoil(sceneGraphAtom);
    let layerPath: TreePath | undefined;
    sceneGraph.forEach((element, i) => {
      visitSceneGraphElement(element, (e, path) => {
        if (e.id === this.layerId) {
          layerPath = [i, ...path];
          return false;
        }
        return undefined;
      });
    });
    if (layerPath) {
      while (layerPath.length > 0) {
        const el = definedAndNotNull(
          findSceneGraphElement(sceneGraph, layerPath)
        );
        if (el.deviceContext) {
          return {
            type: "device",
            deviceId: el.deviceContext,
          };
        }
        layerPath.pop();
      }
    }

    return undefined;
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

  getLayerRoot(layerId?: string): Object3D {
    const f = defined(this.getTransformLayerHelper);
    return f(layerId || this.layerId);
  }

  getLayerParts(layerId: string): {
    [key in string]: Material | Mesh | Group | Object3D | Urdf | undefined;
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

  public createGltf(url: string, onLoad?: () => void): Model3D {
    return new Model3D(url, onLoad);
  }

  public createRoundedRectangle(config: {
    width: number;
    height: number;
    radius: number;
    color?: number;
    smoothness?: number;
  }) {
    const w = config.width;
    const h = config.height;
    const r = config.radius;
    const color = config.color || 0xffffff;
    const smoothness = config.smoothness || 10;
    // https://discourse.threejs.org/t/roundedrectangle/28645
    // helper const's
    const wi = w / 2 - r; // inner width
    const hi = h / 2 - r; // inner height
    const w2 = w / 2; // half width
    const h2 = h / 2; // half height
    const ul = r / w; // u left
    const ur = (w - r) / w; // u right
    const vl = r / h; // v low
    const vh = (h - r) / h; // v high

    const positions = [
      -wi,
      -h2,
      0,
      wi,
      -h2,
      0,
      wi,
      h2,
      0,
      -wi,
      -h2,
      0,
      wi,
      h2,
      0,
      -wi,
      h2,
      0,
      -w2,
      -hi,
      0,
      -wi,
      -hi,
      0,
      -wi,
      hi,
      0,
      -w2,
      -hi,
      0,
      -wi,
      hi,
      0,
      -w2,
      hi,
      0,
      wi,
      -hi,
      0,
      w2,
      -hi,
      0,
      w2,
      hi,
      0,
      wi,
      -hi,
      0,
      w2,
      hi,
      0,
      wi,
      hi,
      0,
    ];

    const uvs = [
      ul,
      0,
      ur,
      0,
      ur,
      1,
      ul,
      0,
      ur,
      1,
      ul,
      1,
      0,
      vl,
      ul,
      vl,
      ul,
      vh,
      0,
      vl,
      ul,
      vh,
      0,
      vh,
      ur,
      vl,
      1,
      vl,
      1,
      vh,
      ur,
      vl,
      1,
      vh,
      ur,
      vh,
    ];

    let phia = 0;
    let phib;
    let xc;
    let yc;
    let uc;
    let vc;
    let cosa;
    let sina;
    let cosb;
    let sinb;

    for (let i = 0; i < smoothness * 4; i += 1) {
      phib = (Math.PI * 2 * (i + 1)) / (4 * smoothness);

      cosa = Math.cos(phia);
      sina = Math.sin(phia);
      cosb = Math.cos(phib);
      sinb = Math.sin(phib);

      xc = i < smoothness || i >= 3 * smoothness ? wi : -wi;
      yc = i < 2 * smoothness ? hi : -hi;

      positions.push(
        xc,
        yc,
        0,
        xc + r * cosa,
        yc + r * sina,
        0,
        xc + r * cosb,
        yc + r * sinb,
        0
      );

      uc = i < smoothness || i >= 3 * smoothness ? ur : ul;
      vc = i < 2 * smoothness ? vh : vl;

      uvs.push(
        uc,
        vc,
        uc + ul * cosa,
        vc + vl * sina,
        uc + ul * cosb,
        vc + vl * sinb
      );

      phia = phib;
    }

    const geometry = new BufferGeometry();
    geometry.setAttribute(
      "position",
      new BufferAttribute(new Float32Array(positions), 3)
    );
    geometry.setAttribute("uv", new BufferAttribute(new Float32Array(uvs), 2));

    return new Mesh(
      geometry,
      new MeshBasicMaterial({
        color,
        side: DoubleSide,
      })
    );
  }

  public createLabel(text: string, sizeAttenuate?: boolean): Label {
    return new Label(text, sizeAttenuate);
  }

  public createText(
    text: string,
    config?: {
      color?: number;
      fontSize?: number;
    }
  ): TextPlane {
    return new TextPlane(text, config?.color || 0xbac4e2, config?.fontSize);
  }

  public createImage(url: string): ImagePlane {
    return new ImagePlane(url);
  }

  protected getField<T extends LayerFieldType>(
    f: LayerField<T>
  ): LayerFieldTypeMap[T] | undefined {
    const layerKey = Object.keys((this.constructor as any).fields).find(
      (key) => (this.constructor as any).fields[key] === f
    );
    if (layerKey) {
      const field = this.layerFields[layerKey];
      if (field) {
        if (field.type !== f.type) {
          throw new Error(`Field ${layerKey} is not a number`);
        }
        // eslint-disable-next-line valid-typeof
        if (typeof field.value !== typeMap[f.type]) {
          throw new Error(`Field ${layerKey} value is not a ${f.type}`);
        }
        return field.value as LayerFieldTypeMap[T];
      }
    }
    return undefined;
  }
}
