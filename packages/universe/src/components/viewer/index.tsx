import { Measure } from "@formant/ui-sdk";
import * as React from "react";
import { Component } from "react";
import { Howler } from "howler";
import * as THREE from "three";
import {
  PointLight,
  Vector3,
  WebGLRenderer,
  HemisphereLight,
  Raycaster,
} from "three";
import styled from "styled-components";
import { RGBELoader } from "../../../three-utils/loaders/RGBELoader";
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
  SceneGraph,
} from "../../model/SceneGraph";
import { TreePath, treePathEquals } from "../../model/ITreeElement";
import { IUniverseData } from "../../model/IUniverseData";
import { Color } from "../../../../common/Color";
import { XRControllerModelFactory } from "../../../three-utils/webxr/XRControllerModelFactory";
import { OculusHandModel } from "../../../three-utils/webxr/OculusHandModel";
import { Hand, HandPose } from "./Hand";
import { Controller } from "./Controller";
import { HandheldController } from "./HandheldController";

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
interface HandHeldGamePadState {
  buttons: number[];
  axes: number[];
}

interface GamePadState extends HandHeldGamePadState {
  handedness: THREE.XRHandedness;
}

export class UniverseViewer extends Component<IUniverseViewerProps> {
  private element: HTMLElement | null = null;

  private renderer: WebGLRenderer | undefined;

  private scene: THREE.Scene;

  private root: THREE.Object3D = new THREE.Object3D();

  private camera: THREE.PerspectiveCamera;

  private pathToLayer: Map<string, TransformLayer> = new Map();

  private editControls: TransformControls | undefined;

  private orbitControls: OrbitControls | undefined;

  private attachedPath: TreePath | undefined;

  private raycaster = new Raycaster();

  private pointer = new THREE.Vector2();

  private pointerDirty = false;

  private isInVR = false;

  private usingHands = false;

  private clock = new THREE.Clock();

  handHeldControllerGamePadState: HandHeldGamePadState | undefined;

  vrControllerGamePads: Map<THREE.XRInputSource, GamePadState> = new Map();

  private currentHandPoses: HandPose[] = ["unknown", "unknown"];

  constructor(props: IUniverseViewerProps) {
    super(props);
    Howler.volume(1);
    this.scene = new THREE.Scene();
    this.scene.add(this.root);
    this.camera = new THREE.PerspectiveCamera(
      75,
      window.innerWidth / window.innerHeight,
      0.1,
      1000
    );
    this.camera.position.z = -1.5;
    this.camera.position.x = 1.5;
    this.camera.position.y = 1;

    new RGBELoader().load(
      "https://threejs.org/examples/textures/equirectangular/venice_sunset_1k.hdr",
      (hdrEquirect: any) => {
        hdrEquirect.mapping = THREE.EquirectangularReflectionMapping;

        this.scene.environment = hdrEquirect;
      }
    );

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
      this.renderer.sortObjects = false;
      this.renderer.xr.enabled = true;
      /* 
      this.renderer.physicallyCorrectLights = true;
      this.renderer.toneMapping = ACESFilmicToneMapping;
      this.renderer.shadowMap.enabled = true;
      */
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

      // create lines coming off controllers
      const controller1 = this.renderer.xr.getController(0);
      this.scene.add(controller1);

      const controller2 = this.renderer.xr.getController(1);
      this.scene.add(controller2);

      const geometry = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(0, 0, 0),
        new THREE.Vector3(0, 0, -1),
      ]);

      const line = new THREE.Line(geometry);
      line.name = "line";
      line.scale.z = 5;

      controller1.add(line.clone());
      controller2.add(line.clone());

      const controllerModelFactory = new XRControllerModelFactory();

      const controllerGrip1 = this.renderer.xr.getControllerGrip(0);
      controllerGrip1.add(
        controllerModelFactory.createControllerModel(controllerGrip1)
      );
      this.scene.add(controllerGrip1);

      const hand1 = this.renderer.xr.getHand(0);
      const handModel1 = new OculusHandModel(hand1) as unknown as Hand;
      hand1.add(handModel1);
      this.scene.add(hand1);

      // Hand 2
      const controllerGrip2 = this.renderer.xr.getControllerGrip(1);
      controllerGrip2.add(
        controllerModelFactory.createControllerModel(controllerGrip2)
      );
      this.scene.add(controllerGrip2);

      const hand2 = this.renderer.xr.getHand(1);
      const handModel2 = new OculusHandModel(hand2) as unknown as Hand;
      hand2.add(handModel2);
      this.scene.add(hand2);

      const hands = [handModel1, handModel2];

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
            const controllers: Controller[] = [];
            session.inputSources.forEach((source, i) => {
              let handedness: THREE.XRHandedness = "none";
              if (source && source.handedness) {
                handedness = source.handedness;
              }
              if (source.gamepad) {
                const controller = renderer.xr.getController(i) as Controller;
                controller.handedness = handedness;
                controller.pulse = function pulse(
                  intensity: number,
                  length: number
                ) {
                  const gamepad = source.gamepad as any;
                  if (
                    gamepad.hapticActuators &&
                    gamepad.hapticActuators.length > 0
                  ) {
                    gamepad.hapticActuators[0].pulse(intensity, length);
                  }
                };
                controllers.push(controller);
                const buttons = source.gamepad.buttons.map((b) => b.value);
                const axes = Array.from(source.gamepad.axes.slice(0));
                const oldState = this.vrControllerGamePads.get(source);
                const newState = {
                  handedness,
                  buttons,
                  axes,
                };
                const raycaster = new Raycaster();

                const controllerTempMatrix = new THREE.Matrix4();
                controllerTempMatrix
                  .identity()
                  .extractRotation(controller.matrixWorld);

                raycaster.ray.origin.setFromMatrixPosition(
                  controller.matrixWorld
                );
                raycaster.ray.direction
                  .set(0, 0, -1)
                  .applyMatrix4(controllerTempMatrix);
                controller.raycaster = raycaster;

                if (handedness === "left") {
                  hands[1].raycaster = raycaster;
                } else if (handedness === "right") {
                  hands[0].raycaster = raycaster;
                }

                if (oldState) {
                  for (let p = 0; p < newState.buttons.length; p += 1) {
                    if (newState.buttons[p] !== oldState.buttons[p]) {
                      this.notifyControllerButtonChanged(
                        controller as Controller,
                        p,
                        newState.buttons[p]
                      );
                    }
                  }

                  for (let p = 0; p < newState.axes.length; p += 1) {
                    if (newState.axes[p] !== oldState.axes[p]) {
                      this.notifyControllerAxisChanged(
                        controller as Controller,
                        p,
                        newState.axes[p]
                      );
                    }
                  }
                }
                this.vrControllerGamePads.set(source, newState);
              }
            });
            if (controllers.length > 0) {
              this.notifyControllers(controllers);
            }
            if (hands.length > 0) {
              if (
                hands[0].controller.visible === true &&
                hands[1].controller.visible === true &&
                this.usingHands === false
              ) {
                this.notifyHandsEnter(hands);
                this.usingHands = true;
              }
              if (
                hands[0].controller.visible !== true &&
                hands[1].controller.visible !== true &&
                this.usingHands === true
              ) {
                this.notifyHandsLeave(hands);
                this.usingHands = false;
                this.currentHandPoses = ["unknown", "unknown"];
                this.notifyHandPosesChanged(hands);
              }
              if (this.usingHands) {
                this.notifyHandsMoved(hands);
              }
              const hand1Pose = hands[0].getHandPose();
              const hand2Pose = hands[1].getHandPose();
              if (
                this.currentHandPoses[0] !== hand1Pose ||
                this.currentHandPoses[1] !== hand2Pose
              ) {
                this.currentHandPoses = [hand1Pose, hand2Pose];
                this.notifyHandPosesChanged(hands);
              }
            }
          }
        } else {
          const gamepad = navigator.getGamepads()[0];
          if (gamepad) {
            const buttons = gamepad.buttons.map((b) => b.value);
            const axes = Array.from(gamepad.axes.slice(0));
            const oldState = this.handHeldControllerGamePadState;
            const newState = {
              buttons,
              axes,
            };
            const handHeld = {
              gamepad,
              vibrate(params: {
                startDelay: number;
                duration: number;
                weakMagnitude: number;
                strongMagnitude: number;
              }) {
                // @ts-ignore-next-line
                const actuator = gamepad.vibrationActuator as any;
                if (actuator) {
                  actuator.playEffect(actuator.type, params);
                }
              },
            };
            if (oldState) {
              for (let p = 0; p < newState.buttons.length; p += 1) {
                if (newState.buttons[p] !== oldState.buttons[p]) {
                  this.notifyHandheldControllerButtonChanged(
                    handHeld as HandheldController,
                    p,
                    newState.buttons[p]
                  );
                }
              }

              for (let p = 0; p < newState.axes.length; p += 1) {
                if (newState.axes[p] !== oldState.axes[p]) {
                  this.notifyHandheldControllerAxisChanged(
                    handHeld as HandheldController,
                    p,
                    newState.axes[p]
                  );
                }
              }
            }
            this.handHeldControllerGamePadState = newState;
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

        Howler.pos(
          this.camera.position.x,
          this.camera.position.y,
          this.camera.position.z
        );
        let lookAt = new Vector3(0, 0, -1);
        let up = new Vector3(0, 1, 0);
        lookAt = lookAt.applyQuaternion(this.camera.quaternion);
        up = up.applyQuaternion(this.camera.quaternion);
        Howler.orientation(lookAt.x, lookAt.y, lookAt.z, up.x, up.y, up.z);
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

  getTransformLayerById = (layerId: string) => {
    const layer = this.pathToLayer.get(layerId);
    return defined(layer);
  };

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

  private notifyControllerButtonChanged(
    controller: Controller,
    button: number,
    value: number
  ) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onControllerButtonChanged(
        controller,
        button,
        value
      );
    });
  }

  private notifyControllerAxisChanged(
    controller: Controller,
    axis: number,
    value: number
  ) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onControllerAxisChanged(controller, axis, value);
    });
  }

  private notifyHandheldControllerButtonChanged(
    controller: HandheldController,
    button: number,
    value: number
  ) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onHandheldControllerButtonChanged(
        controller,
        button,
        value
      );
    });
  }

  private notifyHandheldControllerAxisChanged(
    controller: HandheldController,
    axis: number,
    value: number
  ) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onHandheldControllerAxisChanged(
        controller,
        axis,
        value
      );
    });
  }

  private notifyControllers(controllers: Controller[]) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onControllersMoved(controllers);
    });
  }

  private notifyHandsMoved(hands: Hand[]) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onHandsMoved(hands);
    });
  }

  private notifyHandPosesChanged(hands: Hand[]) {
    Array.from(this.pathToLayer.values()).forEach((_) => {
      defined(_.contentNode).onHandPosesChanged(hands);
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
    const layer = defined(this.pathToLayer.get(el.id));

    defined(layer.contentNode).destroy();
    definedAndNotNull(layer.parent).remove(layer);
    this.pathToLayer.delete(el.id);
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
      this.getCurrentCamera,
      this.getTransformLayerById
    );
    if (el.position.type === "manual") {
      layer.position.set(el.position.x, el.position.y, el.position.z);
    }
    const parent = getSceneGraphElementParent(sceneGraph, path);
    if (parent) {
      const o = defined(this.pathToLayer.get(parent.id));
      o.add(layer);
    } else {
      this.root.add(layer);
    }
    this.pathToLayer.set(el.id, layer);
  }

  public updateLayerVisibility(id: string, visible: boolean) {
    const layer = defined(this.pathToLayer.get(id));
    if (layer.visible !== visible) {
      layer.visible = visible;
      defined(layer.contentNode).onVisibilityChanged(visible);
    }
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
    const o = defined(this.pathToLayer.get(el.id));
    defined(o.contentNode).onFieldChanged(fieldId, value);
  }

  public toggleVisible(
    sceneGraph: SceneGraph,
    path: TreePath,
    visible: boolean
  ) {
    const el = definedAndNotNull(findSceneGraphElement(sceneGraph, path));
    const o = defined(this.pathToLayer.get(el.id));
    o.visible = visible;
    defined(o.contentNode).onVisibilityChanged(visible);
  }

  public toggleEditing(
    sceneGraph: SceneGraph,
    path: TreePath,
    editing: boolean
  ) {
    const c = defined(this.editControls);
    const el = definedAndNotNull(findSceneGraphElement(sceneGraph, path));
    const o = defined(this.pathToLayer.get(el.id));
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
    const o = defined(this.pathToLayer.get(el.id));
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
