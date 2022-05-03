import * as THREE from "three";

import URDFLoader, { URDFRobot } from "urdf-loader";
import { Group, LoadingManager, Mesh, Scene } from "three";
import { ColladaLoader } from "../../three-utils/ColladaLoader";
import { IJointState } from "../../../data-sdk/src/model/IJointState";
import { ITransform } from "../../../model/ITransform";
import { transformMatrix } from "../math/transformMatrix";

export interface ILoadedUrdf {
  url: string;
  robot: URDFRobot;
}

interface UrdfConfig {
  ghosted?: boolean;
  endEffectorLink?: string;
  endEffectorOnly?: boolean;
}
export class Urdf extends Group {
  private robot?: URDFRobot;

  private meshs: Mesh[] = [];

  private opacity = 1.0;

  private transparent = false;

  private color = new THREE.Color("white");

  // eslint-disable-next-line no-use-before-define
  static urdfMap: { [configJson: string]: Urdf } = {};

  static getUrdf(
    deviceId: string,
    config: UrdfConfig,
    url?: string,
    onLoaded?: () => void
  ) {
    const str = `${JSON.stringify(config)}.${deviceId}`;
    if (Urdf.urdfMap[str] === undefined) {
      if (!url) {
        return { urdf: undefined, loaded: false };
      }
      Urdf.urdfMap[str] = new Urdf(url, config, onLoaded);
      return { urdf: Urdf.urdfMap[str], loaded: false };
    }

    return { urdf: Urdf.urdfMap[str], loaded: true };
  }

  constructor(
    url: string,
    private configuration?: UrdfConfig,
    public onLoaded?: () => void
  ) {
    super();
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);
    if (this.configuration?.ghosted) {
      this.color = new THREE.Color("lightgrey");
      this.opacity = 0.25;
      this.transparent = true;
    }
    if (this.configuration?.endEffectorOnly) {
      this.color = new THREE.Color("green");
      this.opacity = 0.005;
      this.transparent = true;
    }
    const { opacity, transparent } = this;
    let { color } = this;

    loader.loadMeshCb = (
      path: string,
      loadingManager: LoadingManager,
      done: (s: Scene) => void
    ) => {
      const daeLoader = new ColladaLoader(loadingManager);
      daeLoader.load(path, (dae) => {
        dae.scene.children = dae.scene.children.filter(
          (_) => !_.type.endsWith("Light")
        );
        dae.scene.traverse((_) => {
          if (_ instanceof Mesh) {
            this.meshs.push(_);
            _.geometry.computeVertexNormals();
            if (
              !this.configuration?.ghosted &&
              !this.configuration?.endEffectorOnly
            ) {
              let mat: THREE.Material;
              if (Array.isArray(_.material)) {
                [mat] = _.material;
              } else {
                mat = _.material;
              }
              if (
                mat instanceof THREE.MeshPhongMaterial ||
                mat instanceof THREE.MeshStandardMaterial ||
                mat instanceof THREE.MeshLambertMaterial ||
                mat instanceof THREE.MeshBasicMaterial
              ) {
                color = mat.color;
              }
            }
            _.material = new THREE.MeshPhongMaterial({
              color,
              opacity,
              transparent,
            });
          }
        });

        done(dae.scene);
      });
    };

    loader.load(url, this.onLoad);
  }

  private onLoad = (robot: URDFRobot) => {
    this.robot = robot;
    if (this.configuration?.endEffectorOnly) {
      if (this.configuration.endEffectorLink) {
        const endEffector =
          this.robot.links[this.configuration.endEffectorLink];
        this.add(endEffector);
      } else {
        console.warn(
          "URDF configured for endEffectorOnly but endEffectorLink not provided"
        );
      }
    } else {
      this.add(this.robot);
    }
    this.onLoaded?.();
  };

  public setColor = (color: THREE.Color) => {
    const { opacity, transparent } = this;
    this.meshs.forEach((_) => {
      _.material = new THREE.MeshPhongMaterial({
        opacity,
        transparent,
        color,
      });
    });
  };

  public set worldToLocalTransform(worldToLocal: ITransform | undefined) {
    if (worldToLocal) {
      this.matrix.copy(transformMatrix(worldToLocal));
    } else {
      this.matrix.identity();
    }
  }

  public set jointState(jointState: IJointState) {
    const { robot } = this;
    if (!robot) {
      return;
    }

    const { name: names } = jointState;

    Object.keys(robot.joints).forEach((_) => {
      const joint = robot.joints[_];

      const index = names.indexOf(_);

      const effort = jointState.effort?.[index] ?? 0;
      const position = jointState.position?.[index] ?? 0;
      const velocity = jointState.velocity?.[index] ?? 0;

      joint.setJointValue(position, effort, velocity);
    });
  }

  public getLink = (name?: string) =>
    name ? this.robot?.links[name] : undefined;
}
