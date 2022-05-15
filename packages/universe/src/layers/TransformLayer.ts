import * as uuid from "uuid";
import { getDistance } from "geolib";
import { Euler, Matrix4, Object3D, Quaternion, Vector3 } from "three";
import { defined } from "../../../common/defined";
import { ITransformNode } from "../../../model/ITransformNode";
import { Positioning } from "../model/SceneGraph";
import { TreePath } from "../model/ITreeElement";
import { IUniverseData } from "../model/IUniverseData";

import { UniverseLayer } from "./UniverseLayer";

export class TransformLayer extends Object3D {
  static layerTypeId: string = "transform_space";

  static commonName = "Empty";

  static description = "An empty layer to fit other layers in.";

  static usesData = false;

  public universeData!: IUniverseData;

  public deviceId?: string;

  contentNode: UniverseLayer | undefined;

  buildTransformList(
    transformNodes: ITransformNode[],
    path: TreePath,
    transformsSoFar?: { pos: Vector3; rotation: Quaternion }[]
  ): { pos: Vector3; rotation: Quaternion }[] {
    const newTransformsSoFar = transformsSoFar || [];
    const i = path.shift();
    if (i === undefined) {
      return newTransformsSoFar;
    }
    const node = transformNodes[i];
    const pos = defined(node.transform).translation;
    const { rotation } = defined(node.transform);
    newTransformsSoFar.push({
      pos: new Vector3(pos.x, pos.y, pos.z),
      rotation: new Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
    });
    if (node.children)
      return this.buildTransformList(node.children, path, newTransformsSoFar);
    return newTransformsSoFar;
  }

  findPathToName(
    transformNodes: ITransformNode[],
    name: string,
    pathSoFar?: TreePath
  ): TreePath {
    const newPathSoFar = pathSoFar || [];

    for (let i = 0; i < transformNodes.length; i += 1) {
      if (transformNodes[i].name === name) {
        newPathSoFar.push(i);
        return newPathSoFar;
      }
    }

    // not found so go down the tree
    for (let i = 0; i < transformNodes.length; i += 1) {
      const { children } = transformNodes[i];
      if (children && children.length > 0) {
        const ret = this.findPathToName(children, name, [...newPathSoFar, i]);
        if (ret.length > 0) {
          return ret;
        }
      }
    }

    return [];
  }

  positionUnsubsciber: undefined | (() => void);

  setPositioning(positioning: Positioning, universeData: IUniverseData) {
    const { deviceId } = this;
    if (this.positionUnsubsciber) {
      this.positionUnsubsciber();
      this.positionUnsubsciber = undefined;
    }

    if (
      positioning.type === "gps" &&
      positioning.stream &&
      positioning.relativeToLongitude !== undefined &&
      positioning.relativeToLatitude !== undefined
    ) {
      this.positionUnsubsciber = universeData.subscribeToLocation(
        defined(deviceId),
        {
          id: uuid.v4(),
          sourceType: "telemetry",
          streamName: positioning.stream,
          streamType: "location",
        },
        (location) => {
          const h1 = {
            longitude: location.longitude,
            latitude: location.latitude,
          };
          const h2 = {
            longitude: positioning.relativeToLongitude,
            latitude: location.latitude,
          };
          const horizontalDistance = getDistance(h1, h2, 0.000001);
          const l1 = {
            longitude: location.longitude,
            latitude: location.latitude,
          };
          const l2 = {
            longitude: location.longitude,
            latitude: positioning.relativeToLatitude,
          };
          const verticalDistance = getDistance(l1, l2, 0.000001);
          const euler = new Euler(0, 0, location.orientation);
          const quaternion = new Quaternion();
          quaternion.setFromEuler(euler);

          this.matrix = new Matrix4().compose(
            new Vector3(
              horizontalDistance,
              location.altitude || 0,
              verticalDistance
            ),
            quaternion,
            new Vector3(1, 1, 1)
          );
          this.matrixAutoUpdate = false;
        }
      );
    } else if (
      positioning.type === "transform tree" &&
      positioning.stream &&
      positioning.end
    ) {
      this.positionUnsubsciber = universeData.subscribeToTransformTree(
        defined(deviceId),
        {
          id: uuid.v4(),
          sourceType: "telemetry",
          streamName: positioning.stream,
          streamType: "transform tree",
        },
        (transformTree) => {
          fetch(defined(transformTree.url))
            .then((res) => res.json())
            .then((tree) => {
              const pathToName = this.findPathToName(
                [tree],
                defined(positioning.end)
              );
              const transforms = this.buildTransformList([tree], pathToName);
              const transformMatrices = transforms.map((_) =>
                new Matrix4().compose(_.pos, _.rotation, new Vector3(1, 1, 1))
              );
              const transformMatrix = transformMatrices.reduce(
                (acc, curr) => acc.multiply(curr),
                new Matrix4()
              );

              this.matrix = transformMatrix;
              this.matrixAutoUpdate = false;
            })
            .catch((err) => {
              throw err;
            });
        }
      );
    } else if (positioning.type === "manual") {
      this.position.set(positioning.x, positioning.y, positioning.z);
    }
  }
}
