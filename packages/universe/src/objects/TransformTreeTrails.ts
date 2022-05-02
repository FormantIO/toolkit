import { Group, Vector2 } from "three";
import { IVector3 } from "../../../data-sdk/src/model/IVector3";
import { transformMatrix } from "../math/transformMatrix";
import { ITransformTreeNode } from "./transformTreeLoader";
import { vector } from "../math/vector";
import { Trails } from "./Trails";

function calculateEnds(node: ITransformTreeNode): { [name: string]: IVector3 } {
  const { name, transform, children } = node;
  if (!children?.length) {
    return {
      [name]: vector(node.transform.translation),
    };
  }

  const results: { [name: string]: IVector3 } = {};
  children.forEach((child) => {
    const ends = calculateEnds(child);
    Object.keys(ends).forEach((childName) => {
      results[childName] = vector(ends[childName]).applyMatrix4(
        transformMatrix(transform)
      );
    });
  });

  return results;
}

function calculateTrails(nodes: ITransformTreeNode[]): {
  [name: string]: IVector3[];
} {
  const results: { [name: string]: IVector3[] } = {};

  nodes.forEach((node) => {
    const ends = calculateEnds(node);
    Object.keys(ends).forEach((name) => {
      results[name] = results[name] || [];
      results[name].push(ends[name]);
    });
  });

  return results;
}

export class TransformTreeTrails extends Group {
  private trails: Trails;

  constructor() {
    super();
    this.trails = new Trails();
    this.add(this.trails);
  }

  public set nodes(nodes: ITransformTreeNode[]) {
    this.trails.vertices = Object.values(calculateTrails(nodes));
  }

  public set resolution(resolution: Vector2) {
    this.trails.resolution = resolution;
  }
}
