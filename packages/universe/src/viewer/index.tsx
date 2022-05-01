import { Measure } from "@formant/ui-sdk";
import * as React from "react";
import { Component } from "react";
import * as THREE from "three";
import { Vector3, WebGLRenderer } from "three";
import styled from "styled-components";
import { OrbitControls } from "../../three-utils/OrbitControls";
import { TransformControls } from "../../three-utils/TransformControls";
import { VRButton } from "../../three-utils/VRButton";
import { defined, definedAndNotNull } from "../../../common/defined";
import { LayerRegistry } from "../layers/LayerRegistry";
import { TransformLayer } from "../layers/TransformLayer";
import { injectLayerFieldValues } from "../layers/UniverseLayerContent";
import {
  findSceneGraphElement,
  getSceneGraphElementParent,
  Positioning,
  SceneGraphElement,
} from "../SceneGraph";
import { TreePath, treePathEquals } from "../ITreeElement";
import { IUniverseData } from "../IUniverseData";

const MeasureContainer = styled.div`
  width: 100%;
  height: 100vh;

  background: #303030;

  > div {
    overflow: hidden;
    width: 100%;
    height: 100%;
  }
`;

export interface IUniverseViewerProps {
  universeData: IUniverseData;
  sceneGraph: SceneGraphElement[];
  onSceneGraphElementEdited: (path: TreePath, transform: Vector3) => void;
  vr?: boolean;
}

export class UniverseViewer extends Component<IUniverseViewerProps> {
  private element: HTMLElement | null = null;

  private renderer: WebGLRenderer | undefined;

  private scene: THREE.Scene;

  private root: THREE.Object3D = new THREE.Object3D();

  private camera: THREE.PerspectiveCamera;

  private pathToLayer: Map<SceneGraphElement, TransformLayer<any>> = new Map();

  private editControls: TransformControls | undefined;

  private orbitControls: OrbitControls | undefined;

  private attachedPath: TreePath | undefined;

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
    this.camera.position.z = 3;

    this.camera.position.y = 3;

    const light = new THREE.AmbientLight(0x555555);
    this.scene.add(light);
  }

  public componentWillUnmount() {
    window.removeEventListener("keydown", this.onKeyDown);
  }

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
    }
  };

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
      const { devicePixelRatio } = window;
      const { offsetWidth: width, offsetHeight: height } = element;
      this.renderer.setPixelRatio(devicePixelRatio);
      this.renderer.setSize(width, height);

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
        if (this.renderer && this.orbitControls) {
          this.renderer.render(this.scene, this.camera);
          this.orbitControls.update();
        }
      });
      if (vr) element.appendChild(VRButton.createButton(this.renderer));
    }
  }

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

  public recenter() {
    if (this.orbitControls) {
      this.orbitControls.reset();
    }
  }

  getCurrentCamera = () => this.camera;

  public addSceneGraphItem(path: TreePath, deviceId?: string) {
    const el = definedAndNotNull(
      findSceneGraphElement(this.props.sceneGraph, path)
    );

    const fields = LayerRegistry.getFields(el.type);
    injectLayerFieldValues(fields, el.fieldValues);
    const layer = LayerRegistry.createDefaultLayer(
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
    const parent = getSceneGraphElementParent(this.props.sceneGraph, path);
    if (parent) {
      const o = defined(this.pathToLayer.get(parent));
      o.add(layer);
    } else {
      this.root.add(layer);
    }
    this.pathToLayer.set(el, layer);
  }

  public removeSceneGraphItem(path: TreePath) {
    if (this.attachedPath && treePathEquals(path, this.attachedPath)) {
      this.toggleEditing(path, false);
    }
    const el = definedAndNotNull(
      findSceneGraphElement(this.props.sceneGraph, path)
    );
    const layer = defined(this.pathToLayer.get(el));

    definedAndNotNull(layer.parent).remove(layer);
    this.pathToLayer.delete(el);
  }

  private onResize = (width: number, height: number) => {
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    if (this.orbitControls) {
      this.orbitControls.update();
    }
    this.renderer?.setSize(width, height);
  };

  public toggleVisible(path: TreePath, visible: boolean) {
    const el = definedAndNotNull(
      findSceneGraphElement(this.props.sceneGraph, path)
    );
    const o = defined(this.pathToLayer.get(el));
    o.visible = visible;
    o.traverse((child) => {
      child.visible = visible;
    });
  }

  public toggleEditing(path: TreePath, editing: boolean) {
    const c = defined(this.editControls);
    const el = definedAndNotNull(
      findSceneGraphElement(this.props.sceneGraph, path)
    );
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

  public updatePositioning(path: TreePath, position: Positioning) {
    const el = definedAndNotNull(
      findSceneGraphElement(this.props.sceneGraph, path)
    );
    const o = defined(this.pathToLayer.get(el));
    o.setPositioning(position, this.props.universeData);
  }

  public render() {
    return (
      <MeasureContainer>
        <Measure onResize={this.onResize}>
          <div ref={(_) => (this.element = defined(_))} />
        </Measure>
      </MeasureContainer>
    );
  }
}
