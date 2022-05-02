import {
  BufferGeometry,
  Color,
  EllipseCurve,
  Group,
  Line,
  LineBasicMaterial,
  Mesh,
  Path,
} from "three";
import { range } from "../../../common/range";

function polarGridCircle(
  radius: number,
  circleColor: Color,
  points: number = 36
) {
  const curve = new EllipseCurve(
    0,
    0,
    radius,
    radius,
    0,
    2 * Math.PI,
    false,
    0
  );
  const path = new Path(curve.getPoints(points));
  const geometry = new BufferGeometry().setFromPoints(path.getPoints());
  const material = new LineBasicMaterial({
    color: circleColor,
  });
  return new Line(geometry, material);
}

function polarGrid(majorCircleColor: Color, minorCircleColor: Color) {
  const mesh = new Mesh();

  range(-1, 2).forEach((magnitude, index) => {
    const first = index === 0;
    range(first ? 1 : 3, 21).forEach((i) => {
      const major = i === 10;
      const r = 10 ** magnitude * i;
      mesh.add(
        polarGridCircle(r, major ? majorCircleColor : minorCircleColor, 36)
      );
    });
  });

  return mesh;
}

export class GroundPlane extends Group {
  constructor(
    color2: Color = new Color(0x2d3753),
    color3: Color = new Color(0x3b4569)
  ) {
    super();
    this.add(polarGrid(color3, color2));
  }
}
