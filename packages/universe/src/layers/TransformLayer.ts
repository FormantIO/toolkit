import * as uuid from "uuid";
import { getDistance } from "geolib";
import { Euler, Matrix4, Object3D, Quaternion, Vector3 } from "three";
import { defined } from "../../../common/defined";
import { ITransformNode } from "../../../model/ITransformNode";
import { Positioning } from "../SceneGraph";
import { TreePath } from "../ITreeElement";
import { IUniverseData, UniverseDataSource } from "../IUniverseData";

import { UniverseLayerContent } from "./UniverseLayerContent";

export class TransformLayer<T extends Object3D> extends UniverseLayerContent {
  static id = "transform_space";

  static commonName = "Empty";

  static description = "An empty layer to fit other layers in.";

  static usesData = false;

  static createDefault(
    _universeData: IUniverseData,
    _deviceId: string,
    _universeDataSources?: UniverseDataSource[]
  ): TransformLayer<Object3D> {
    return new TransformLayer();
  }

  contentNode: T | undefined;

  constructor(content?: T) {
    super();
    if (content) {
      this.contentNode = content;
      this.add(content);
    }
  }

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
          const horizontalDistance = getDistance(h1, h2);
          const l1 = {
            longitude: location.longitude,
            latitude: location.latitude,
          };
          const l2 = {
            longitude: location.longitude,
            latitude: positioning.relativeToLatitude,
          };
          const verticalDistance = getDistance(l1, l2);
          const euler = new Euler(0, 0, location.orientation);
          const quaternion = new Quaternion();
          quaternion.setFromEuler(euler);

          this.matrix = new Matrix4().compose(
            new Vector3(
              horizontalDistance,
              verticalDistance,
              location.altitude || 0
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
