import { Group, Vector2 } from "three";
import { IVector3 } from "../../../../data-sdk/src/model/IVector3";
import { Trail } from "./Trail";
import { Color } from "../../../../common/Color";
import { defined } from "../../../../common/defined";

export const colors = [
  "#20a0ff", // royal-blue
  "#ea719d", // red
  "#795bce", // red-dark
  "#f89973", // orange
  "#f9c36e", // yellow
  "#2ec495", // green
  "#64d7d4", // mint
  "#9a8261", // yellow-dark
  "#94645f", // orange-dark
  "#7f5072", // red-dark
  "#564a94", // purple-dark
  "#256faf", // royal-blue-dark
  "#4a8d98", // mint-dark
  "#2d8376", // green-dark
].map((_) => defined(Color.fromString(_)));

export class Trails extends Group {
  private trailsResolution = new Vector2();

  private trails: Trail[] = [];

  public set vertices(vertices: IVector3[][]) {
    for (let i = vertices.length; i < this.trails.length; i += 1) {
      this.remove(this.trails[i]);
      delete this.trails[i];
    }

    for (let i = 0; i < vertices.length; i += 1) {
      if (!this.trails[i]) {
        this.trails[i] = new Trail(colors[i]);
        this.trails[i].resolution = this.trailsResolution;
        this.add(this.trails[i]);
      }
      this.trails[i].vertices = vertices[i].slice(-10);
    }
  }

  public set resolution(resolution: Vector2) {
    this.trailsResolution = resolution;
    Object.values(this.trails).forEach((trail) => {
      trail.resolution = resolution;
    });
  }
}
