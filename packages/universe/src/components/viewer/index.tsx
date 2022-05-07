import { Measure } from "@formant/ui-sdk";
import * as React from "react";
import { Component } from "react";
import * as THREE from "three";
import {
  PointLight,
  Vector3,
  WebGLRenderer,
  HemisphereLight,
  Raycaster,
  XRInputSource,
} from "three";
import styled from "styled-components";
import { OrbitControls } from "../../../three-utils/controls/OrbitControls";
import { TransformControls } from "../../../three-utils/controls/TransformControls";
import { VRButton } from "../../../three-utils/webxr/VRButton";
import { defined, definedAndNotNull } from "../../../../common/defined";
import { LayerRegistry } from "../../layers/LayerRegistry";
import { TransformLayer } from "../../layers/TransformLayer";
import { injectLayerFieldValues } from "../../model/LayerField";
import {
  findSceneGraphElement,
  getSceneGraphElementParent,
  Positioning,
  SceneGraphElement,
  SceneGraph,
} from "../../model/SceneGraph";
import { TreePath, treePathEquals } from "../../model/ITreeElement";
import { IUniverseData } from "../../model/IUniverseData";
import { Color } from "../../../../common/Color";
import { XRControllerModelFactory } from "../../../three-utils/webxr/XRControllerModelFactory";
import { OculusHandModel } from "../../../three-utils/webxr/OculusHandModel";
import { Hand } from "./Hand";

const MeasureContainer = styled.div`
  width: 100%;
  height: 100vh;

  background: #1c1e2c;

  > div {
    overflow: hidden;
    width: 100%;
    height: 100%;
  }
`;

export interface IUniverseViewerProps {
  universeData: IUniverseData;
  onSceneGraphElementEdited: (path: TreePath, transform: Vector3) => void;
  vr?: boolean;
}
interface GamePadState {
  handedness: THREE.XRHandedness;
  buttons: number[];
  axes: number[];
}

export class UniverseViewer extends Component<IUniverseViewerProps> {
  private element: HTMLElement | null = null;

  private renderer: WebGLRenderer | undefined;

  private scene: THREE.Scene;

  private root: THREE.Object3D = new THREE.Object3D();

  private camera: THREE.PerspectiveCamera;

  private pathToLayer: Map<SceneGraphElement, TransformLayer> = new Map();

  private editControls: TransformControls | undefined;

  private orbitControls: OrbitControls | undefined;

  private attachedPath: TreePath | undefined;

  private raycaster = new Raycaster();

  private pointer = new THREE.Vector2();

  private pointerDirty = false;

  private isInVR = false;

  private usingHands = false;

  private clock = new THREE.Clock();

  gamePads: Map<THREE.XRInputSource, GamePadState> = new Map();

  constructor(props: IUniverseViewerProps) {
    super(props);
    this.scene = new THREE.Scene();
    this.scene.add(this.root);
    const ninetyDegrees = Math.PI / 2;
    this.root.rotation.set(-ninetyDegrees, 0, 0);
    this.camera = new THREE.PerspectiveCamera(
      75,
      window.innerWidth / window.innerHeight,
      0.1,
      1000
    );
    this.camera.position.z = -1.5;
    this.camera.position.x = 1.5;
    this.camera.position.y = 1;

    const accentColor1 = defined(Color.fromString("#18d2ff")).toString();
    const accentColor2 = defined(Color.fromString("#ea719d")).toString();
    const skyColor = defined(Color.fromString("#f8f9fc")).toString();
    const groundColor = defined(Color.fromString("#282f45")).toString();

    const accentLight1 = new PointLight(accentColor1, 0.3, 0, 0);
    accentLight1.position.set(1000, 1000, 1000);
    this.scene.add(accentLight1);

    const accentLight2 = new PointLight(accentColor2, 0.7, 0, 0);
    accentLight2.position.set(-1000, -1000, 1000);
    this.scene.add(accentLight2);

    const ambientLight = new HemisphereLight(skyColor, groundColor, 0.5);
    this.scene.add(ambientLight);
  }

  public componentDidMount() {
    const { element } = this;
    const { vr } = this.props;
    if (element) {
      this.renderer = new WebGLRenderer({
        alpha: true,
        antialias: true,
      });
      this.renderer.xr.enabled = true;
      element.appendChild(this.renderer.domElement);

      element.addEventListener("pointermove", this.onPointerMove);
      element.addEventListener("pointerdown", this.onPointerDown);
      element.addEventListener("pointerup", this.onPointerUp);
      element.addEventListener("pointerenter", this.onPointerEnter);
      element.addEventListener("pointerleave", this.onPointerLeave);
      element.addEventListener("wheel", this.onPointerWheel);

      const { devicePixelRatio } = window;
      const { offsetWidth: width, offsetHeight: height } = element;
      this.renderer.setPixelRatio(devicePixelRatio);
      this.renderer.setSize(width, height);

      const controllerModelFactory = new XRControllerModelFactory();

      const controllerGrip1 = this.renderer.xr.getControllerGrip(0);
      controllerGrip1.add(
        controllerModelFactory.createControllerModel(controllerGrip1)
      );
      this.scene.add(controllerGrip1);

      const hand1 = this.renderer.xr.getHand(0);
      const handModel1 = new OculusHandModel(hand1);
      hand1.add(handModel1);
      this.scene.add(hand1);

      // Hand 2
      const controllerGrip2 = this.renderer.xr.getControllerGrip(1);
      controllerGrip2.add(
        controllerModelFactory.createControllerModel(controllerGrip2)
      );
      this.scene.add(controllerGrip2);

      const hand2 = this.renderer.xr.getHand(1);
      const handModel2 = new OculusHandModel(hand2);
      hand2.add(handModel2);
      this.scene.add(hand2);

      const hands = [
        handModel1 as unknown as Hand,
        handModel2 as unknown as Hand,
      ];

      this.orbitControls = new OrbitControls(
        this.camera,
        this.renderer.domElement
      );
      this.orbitControls.update();
      this.editControls = new TransformControls(
        this.camera,
        this.renderer.domElement
      );
      window.addEventListener("keydown", this.onKeyDown);
      this.editControls.addEventListener("change", this.onEditControlsChange);
      this.editControls.addEventListener("dragging-changed", (event) => {
        if (this.orbitControls) this.orbitControls.enabled = !event.value;
      });
      this.scene.add(this.editControls);
      this.renderer.setAnimationLoop(() => {
        const renderer = defined(this.renderer);
        if (renderer.xr.isPresenting !== this.isInVR) {
          this.isInVR = renderer.xr.isPresenting;
          if (this.isInVR) {
            if (renderer.xr.setFoveation) renderer.xr.setFoveation(0);
            this.notifyEnterVR();
          } else {
            this.notifyExitVR();
          }
        }
        if (this.isInVR) {
          const session = renderer.xr.getSession();
          if (session) {
            const controllers: THREE.Group[] = [];
            session.inputSources.forEach((source, i) => {
              let handedness: THREE.XRHandedness = "none";
              if (source && source.handedness) {
                handedness = source.handedness;
              }
              if (source.gamepad) {
                const controller = renderer.xr.getController(i);
                controllers.push(controller);
                const buttons = source.gamepad.buttons.map((b) => b.value);
                const axes = Array.from(source.gamepad.axes.slice(0));
                const oldState = this.gamePads.get(source);
                const newState = {
                  handedness,
                  buttons,
                  axes,
                };

                if (oldState) {
                  for (let p = 0; p < newState.buttons.length; p += 1) {
                    if (newState.buttons[p] !== oldState.buttons[p]) {
                      this.notifyGamePadButtonChanged(
                        controller,
                        p,
                        newState.buttons[p],
                        source
                      );
                    }
                  }

                  for (let p = 0; p < newState.axes.length; p += 1) {
                    if (newState.axes[p] !== oldState.axes[p]) {
                      this.notifyGamePadAxisChanged(
                        controller,
                        p,
                        newState.axes[p],
                        source
                      );
                    }
                  }
                }
                this.gamePads.set(source, newState);
              }
            });
            this.notifyControllers(controllers);
            if (
              hands[0].visible &&
              hands[1].visible &&
              this.usingHands === false
            ) {
              this.notifyHandsEnter(hands);
              this.usingHands = true;
            }
            if (
              hands[0].visible === false &&
              hands[1].visible === false &&
              this.usingHands === true
            ) {
              this.notifyHandsLeave(hands);
              this.usingHands = false;
            }
            this.notifyHandsMoved(hands);
          }
        }

        this.raycaster.setFromCamera(this.pointer, this.camera);
        if (this.pointerDirty) {
          this.notifyRaycasterChanged();
          this.pointerDirty = false;
        }
        if (this.orbitControls) {
          renderer.render(this.scene, this.camera);
          this.orbitControls.update();
        }

        this.notifyUpdate(this.clock.getDelta());
      });
      if (vr) element.appendChild(VRButton.createButton(this.renderer));
    }
  }

  public componentWillUnmount() {
    window.removeEventListener("keydown", this.onKeyDown);
  }

  onPointerMove = (event: PointerEvent) => {
    if (this.renderer) {
      this.pointer.x =
        (event.offsetX / this.renderer.domElement.offsetWidth) * 2 - 1;
      this.pointer.y =
        -(event.offsetY / this.renderer.domElement.offsetHeight) * 2 + 1;
      this.pointerDirty = true;
    }
  };

  onPointerDown = (event: PointerEvent) => {
    const { button } = event;
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onPointerDown(this.raycaster, button);
    });
  };

  onPointerUp = (event: PointerEvent) => {
    const { button } = event;
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onPointerUp(this.raycaster, button);
    });
  };

  onPointerEnter = (_event: PointerEvent) => {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onPointerEnter(this.raycaster);
    });
  };

  onPointerLeave = (_event: PointerEvent) => {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onPointerLeave(this.raycaster);
    });
  };

  onPointerWheel = (event: WheelEvent) => {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onPointerWheel(this.raycaster, event.deltaY);
    });
  };

  onKeyDown = (event: KeyboardEvent) => {
    const c = defined(this.editControls);
    switch (event.key) {
      case "g":
        c.setMode("translate");
        break;
      case "r":
        c.setMode("rotate");
        break;
      case "s":
        c.setMode("scale");
        break;
      default:
        break;
    }
  };

  private onEditControlsChange = () => {
    const { editControls } = this;
    if (editControls) {
      const { attachedPath } = this;
      if (attachedPath) {
        const { object } = editControls;
        if (object) {
          const { position } = object;

          this.props.onSceneGraphElementEdited(attachedPath, position);
        }
      }
    }
  };

  getCurrentCamera = () => this.camera;

  private onResize = (width: number, height: number) => {
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    if (this.orbitControls) {
      this.orbitControls.update();
    }
    this.renderer?.setSize(width, height);
  };

  private notifyHandsEnter(hands: Hand[]) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onHandsEnter(hands);
    });
  }

  private notifyHandsLeave(hands: Hand[]) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onHandsLeave(hands);
    });
  }

  private notifyRaycasterChanged() {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onPointerMove(this.raycaster);
    });
  }

  private notifyGamePadButtonChanged(
    controller: THREE.Group,
    button: number,
    value: number,
    source: XRInputSource
  ) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onGamePadButtonChanged(
        controller,
        button,
        value,
        source
      );
    });
  }

  private notifyGamePadAxisChanged(
    controller: THREE.Group,
    axis: number,
    value: number,
    source: XRInputSource
  ) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onGamePadAxisChanged(
        controller,
        axis,
        value,
        source
      );
    });
  }

  private notifyControllers(controllers: THREE.Group[]) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onControllersMoved(controllers);
    });
  }

  private notifyHandsMoved(hands: Hand[]) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onHandsMoved(hands);
    });
  }

  private notifyUpdate(delta: number) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onUpdate(delta);
    });
  }

  private notifyEnterVR() {
    const { xr } = defined(this.renderer);
    this.scene.add(xr.getController(0));
    this.scene.add(xr.getControllerGrip(0));
    this.scene.add(xr.getController(1));
    this.scene.add(xr.getControllerGrip(1));
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onEnterVR(xr);
    });
  }

  private notifyExitVR() {
    const { xr } = defined(this.renderer);
    this.scene.remove(xr.getController(0));
    this.scene.remove(xr.getControllerGrip(0));
    this.scene.remove(xr.getController(1));
    this.scene.remove(xr.getControllerGrip(1));
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onExitVR(xr);
    });
  }

  public removeSceneGraphItem(sceneGraph: SceneGraph, path: TreePath) {
    if (this.attachedPath && treePathEquals(path, this.attachedPath)) {
      this.toggleEditing(sceneGraph, path, false);
    }
    const el = definedAndNotNull(findSceneGraphElement(sceneGraph, path));
    const layer = defined(this.pathToLayer.get(el));

    defined(layer.contentNode).destroy();
    definedAndNotNull(layer.parent).remove(layer);
    this.pathToLayer.delete(el);
  }

  public addSceneGraphItem(
    sceneGraph: SceneGraph,
    path: TreePath,
    deviceId?: string
  ) {
    const el = definedAndNotNull(findSceneGraphElement(sceneGraph, path));

    const fields = LayerRegistry.getFields(el.type);
    injectLayerFieldValues(fields, el.fieldValues);
    const layer = LayerRegistry.createDefaultLayer(
      el.id,
      el.type,
      this.props.universeData,
      deviceId,
      el.dataSources,
      fields,
      this.getCurrentCamera
    );
    if (el.position.type === "manual") {
      layer.position.set(el.position.x, el.position.y, el.position.z);
    }
    const parent = getSceneGraphElementParent(sceneGraph, path);
    if (parent) {
      const o = defined(this.pathToLayer.get(parent));
      o.add(layer);
    } else {
      this.root.add(layer);
    }
    this.pathToLayer.set(el, layer);
  }

  public recenter() {
    if (this.orbitControls) {
      this.orbitControls.reset();
    }
  }

  public notifyFieldChanged(
    sceneGraph: SceneGraph,
    path: TreePath,
    fieldId: string,
    value: string
  ) {
    const el = definedAndNotNull(findSceneGraphElement(sceneGraph, path));
    const o = defined(this.pathToLayer.get(el));
    defined(o.contentNode).onFieldChanged(fieldId, value);
  }

  public toggleVisible(
    sceneGraph: SceneGraph,
    path: TreePath,
    visible: boolean
  ) {
    const el = definedAndNotNull(findSceneGraphElement(sceneGraph, path));
    const o = defined(this.pathToLayer.get(el));
    o.visible = visible;
    defined(o.contentNode).onVisibilityChanged(visible);
    o.traverse((child) => {
      child.visible = visible;
    });
  }

  public toggleEditing(
    sceneGraph: SceneGraph,
    path: TreePath,
    editing: boolean
  ) {
    const c = defined(this.editControls);
    const el = definedAndNotNull(findSceneGraphElement(sceneGraph, path));
    const o = defined(this.pathToLayer.get(el));
    if (editing) {
      c.setMode("translate");
      c.attach(o);
      this.attachedPath = path;
    } else {
      c.detach();
      this.attachedPath = undefined;
    }
  }

  public updatePositioning(
    sceneGraph: SceneGraph,
    path: TreePath,
    position: Positioning
  ) {
    const el = definedAndNotNull(findSceneGraphElement(sceneGraph, path));
    const o = defined(this.pathToLayer.get(el));
    o.setPositioning(position, this.props.universeData);
  }

  public render() {
    return (
      <MeasureContainer>
        <Measure onResize={this.onResize}>
          <div
            ref={(_) => {
              this.element = defined(_);
            }}
          />
        </Measure>
      </MeasureContainer>
    );
  }
}
