import { Group, PerspectiveCamera, Vector2 } from "three";
import { ITransformTreeNode } from "./transformTreeLoader";
import { Node } from "./Node";
import { TransformTreeTrails } from "./TransformTreeTrails";

export class TransformTree extends Group {
  private transformTreeResolution: Vector2 = new Vector2(0, 0);

  private trails: TransformTreeTrails;

  private root?: Node;

  constructor(private camera: PerspectiveCamera) {
    super();

    this.trails = new TransformTreeTrails();
    this.trails.resolution = this.transformTreeResolution;
    this.add(this.trails);
  }

  public set nodes(nodes: ITransformTreeNode[]) {
    this.trails.nodes = nodes;

    const current = nodes.slice(-1)?.[0];
    if (current) {
      let { root } = this;
      if (!root) {
        root = new Node(this.camera, "root");
        root.resolution = this.transformTreeResolution;
        this.add(root);
        this.root = root;
      }
      root.node = current;
    } else {
      const { root } = this;
      if (root) {
        this.remove(root);
        this.root = undefined;
      }
    }
  }

  public set resolution(resolution: Vector2) {
    this.transformTreeResolution = resolution;
    this.trails.resolution = resolution;
    if (this.root) {
      this.root.resolution = resolution;
    }
  }

  public update() {
    if (this.root) {
      this.root.update();
    }
  }
}
