import { v4 as uuid } from "uuid";
import { UniverseDataSource } from "@formant/universe-core";
import { LayerType } from "../layers";
import { LayerFieldValues } from "./LayerField";
import { TreePath } from "./ITreeElement";

export type Positioning =
  | {
      type: "manual";
      x: number;
      y: number;
      z: number;
    }
  | {
      type: "hud";
      x: number;
      y: number;
    }
  | {
      type: "transform tree";
      stream?: string;
      end?: string;
    }
  | {
      type: "localization";
      stream?: string;
      rtcStream?: string;
    }
  | {
      type: "gps";
      stream?: string;
      relativeToLongitude: number;
      relativeToLatitude: number;
    };
export class SceneGraphElement {
  // eslint-disable-next-line no-use-before-define
  children: SceneGraphElement[] = [];

  public id: string;

  public visible: boolean = true;

  public editing: boolean = false;

  public position: Positioning = {
    type: "manual",
    x: 0,
    y: 0,
    z: 0,
  };

  public scale?: {
    x: number;
    y: number;
    z: number;
  };

  public fieldValues: LayerFieldValues = {};

  constructor(
    public name: string,
    public type: LayerType,
    public data: any,
    public dataSources?: UniverseDataSource[],
    public deviceContext?: string
  ) {
    this.id = uuid();
  }
}

export function visitSceneGraphElement(
  sceneGraph: SceneGraphElement,
  visitor: (el: SceneGraphElement, path: TreePath) => boolean | void,
  pathSoFar?: TreePath
) {
  const p = pathSoFar || [];
  const r = visitor(sceneGraph, p);
  if (r !== false && sceneGraph.children.length > 0) {
    sceneGraph.children.forEach((e, i) => {
      visitSceneGraphElement(e, visitor, [...p, i]);
    });
  }
}

export function cloneSceneGraph(
  scenegraph: SceneGraphElement
): SceneGraphElement {
  const c = JSON.parse(JSON.stringify(scenegraph)) as SceneGraphElement;

  visitSceneGraphElement(c, (e) => {
    e.id = uuid();
    e.visible = true;
    e.editing = false;
  });
  return c as SceneGraphElement;
}

export function visitSceneGraphElementReverse(
  sceneGraph: SceneGraphElement,
  visitor: (el: SceneGraphElement, path: TreePath) => void,
  pathSoFar?: TreePath
) {
  const p = pathSoFar || [];
  if (sceneGraph.children.length > 0) {
    sceneGraph.children.forEach((e, i) => {
      visitSceneGraphElementReverse(e, visitor, [...p, i]);
    });
  }
  visitor(sceneGraph, p);
}

export function findSceneGraphElement(
  sceneGraph: SceneGraphElement[],
  path: TreePath
): SceneGraphElement | null {
  if (path.length === 0) {
    return null;
  }
  const child = sceneGraph[path[0]];
  if (!child) {
    return null;
  }
  const nextPath = path.slice(1);
  if (nextPath.length === 0) {
    return child;
  }
  return findSceneGraphElement(child.children, nextPath);
}

export function getSceneGraphElementParent(
  items: SceneGraphElement[],
  path: TreePath
): SceneGraphElement | null {
  if (path.length === 0) {
    return null;
  }
  const parentPath = path.slice(0, path.length - 1);
  const parent = findSceneGraphElement(items, parentPath);
  return parent;
}

export function findSceneGraphParentElement(
  sceneGraph: SceneGraphElement[],
  path: TreePath,
  filter: (el: SceneGraphElement) => boolean
): SceneGraphElement | null {
  if (path.length === 0) {
    return null;
  }
  const parentPath = path.slice(0, path.length - 1);
  const parent = findSceneGraphElement(sceneGraph, parentPath);
  if (!parent) {
    return null;
  }
  if (filter(parent)) {
    return parent;
  }
  return findSceneGraphParentElement(parent.children, path, filter);
}

export type SceneGraph = SceneGraphElement[];
