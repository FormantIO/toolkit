import { Group, Matrix4, PerspectiveCamera, Vector2, Vector3 } from "three";
import { Line2 } from "../../three-utils/lines/Line2";
import { LineGeometry } from "../../three-utils/lines/LineGeometry";
import { LineMaterial } from "../../three-utils/lines/LineMaterial";
import { transformMatrix } from "../math/transformMatrix";
import { ITransformTreeNode } from "./transformTreeLoader";
import { vector } from "../math/vector";
import { Joint } from "./Joint";
import { Pointer } from "./Pointer";

export class Node extends Group {
  private nodeResolution = new Vector2();

  private lineGeometry = new LineGeometry();

  private lineMaterial = new LineMaterial({
    color: 0xbac4e2,
    linewidth: 1.5,
  });

  private joint: Joint;

  private pointer?: Pointer;

  private line: Line2 = new Line2(this.lineGeometry, this.lineMaterial);

  // eslint-disable-next-line no-use-before-define
  private nodes: { [key: string]: Node } = {};

  constructor(private camera: PerspectiveCamera, root?: "root") {
    super();
    this.matrixAutoUpdate = false;
    this.joint = new Joint(camera);
    if (!root) {
      this.add(this.joint);
      this.add(this.line);
    }
  }

  public set node(node: ITransformTreeNode) {
    const { transform, children } = node;
    const currentNames = Object.keys(this.nodes);
    const newNames = (node.children || []).map((_) => _.name);

    const removed = currentNames.filter((_) => !newNames.includes(_));
    removed.forEach((name) => {
      this.remove(this.nodes[name]);
      delete this.nodes[name];
    });

    const matrix = transformMatrix(transform);

    newNames.forEach((name) => {
      let child = this.nodes[name];
      if (!child) {
        child = new Node(this.camera);
        child.resolution = this.nodeResolution;
        this.add(child);
        this.nodes[name] = child;
      }
      const childNode = children?.find((_) => _.name === name);
      if (childNode) {
        child.node = childNode;
      }
      child.matrix.copy(matrix);
    });

    const lineVertices = [new Vector3(), vector(node.transform.translation)];
    this.lineGeometry = new LineGeometry();
    this.lineGeometry.setPositions(
      lineVertices.map(({ x, y, z }) => [x, y, z]).flat()
    );
    this.line.geometry = this.lineGeometry;

    this.terminal =
      !children?.length && !matrix.equals(new Matrix4().identity());

    if (this.pointer) {
      this.pointer.matrix.copy(matrix);
    }
  }

  private set terminal(terminal: boolean) {
    if (terminal === !!this.pointer) {
      return;
    }
    if (!terminal && this.pointer) {
      this.remove(this.pointer);
    }
    if (terminal) {
      this.pointer = new Pointer(this.camera);
      this.pointer.resolution = this.nodeResolution;
      this.add(this.pointer);
    }
  }

  public set resolution(resolution: Vector2) {
    this.nodeResolution = resolution;
    this.lineMaterial.resolution = resolution;
    this.joint.resolution = resolution;

    if (this.pointer) {
      this.pointer.resolution = resolution;
    }

    Object.values(this.nodes).forEach((child) => {
      child.resolution = resolution;
    });
  }

  public update() {
    this.joint.update();

    if (this.pointer) {
      this.pointer.update();
    }

    Object.values(this.nodes).forEach((child) => {
      child.update();
    });
  }
}
