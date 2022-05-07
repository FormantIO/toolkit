import { Group, Vector2 } from "three";
import { Line2 } from "../../three-utils/lines/Line2";
import { LineGeometry } from "../../three-utils/lines/LineGeometry";
import { LineMaterial } from "../../three-utils/lines/LineMaterial";
import { IVector3 } from "../../../data-sdk/src/model/IVector3";
import { Color } from "../../../common/Color";

export class Trail extends Group {
  private material = new LineMaterial({
    transparent: true,
    color: 0x18d2ff,
    linewidth: 1.5,
    depthTest: false,
    dashScale: 100,
    dashSize: 1,
    gapSize: 0.5,
  });

  private line?: Line2;

  constructor(color?: Color) {
    super();
    if (color) {
      const { r, g, b } = color;
      this.material.color.setRGB(r / 255, g / 255, b / 255);
    }
    Object.defineProperty(this, "renderOrder", {
      get() {
        if (this.line) {
          return this.line.renderOrder;
        }
        return super.renderOrder;
      },
      set(value) {
        if (this.line) {
          this.line.renderOrder = value;
        }
      },
    });
  }

  public set vertices(vertices: IVector3[]) {
    const geometry = new LineGeometry();
    this.material.uniforms.opacity.value = 0.5;
    this.material.defines.USE_DASH = "";
    if (vertices.length > 1) {
      if (!this.line) {
        this.line = new Line2(geometry, this.material);
        this.add(this.line);
      }
      geometry.setPositions(vertices.map(({ x, y, z }) => [x, y, z]).flat());
      this.line.geometry = geometry;
    } else if (this.line) {
      this.remove(this.line);
      this.line = undefined;
    }
    this.line?.computeLineDistances();
  }

  public set resolution(resolution: Vector2) {
    this.material.resolution = resolution;
  }
}
