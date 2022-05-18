import {
  BufferAttribute,
  BufferGeometry,
  Color,
  Group,
  Line,
  LineBasicMaterial,
  Vector3,
} from "three";

function axisLine(vector: Vector3, color: Color, opacity: number = 0.4) {
  const material = new LineBasicMaterial({
    color,
    opacity,
    depthTest: true,
    depthWrite: true,
  });
  material.transparent = true;
  const geometry = new BufferGeometry();
  const vertices = new Float32Array([0, 0, 0, vector.x, vector.y, vector.z]);
  geometry.setAttribute("position", new BufferAttribute(vertices, 3));
  return new Line(geometry, material);
}

export class Axes extends Group {
  constructor(
    flat: boolean,
    xColor: Color = new Color(0xea719d),
    yColor: Color = new Color(0x2ec495),
    zColor: Color = new Color(0x18d2ff)
  ) {
    super();

    const radius = 1000;

    this.add(axisLine(new Vector3(radius, 0, 0), xColor));
    this.add(axisLine(new Vector3(-radius, 0, 0), xColor));

    if (!flat) {
      this.add(axisLine(new Vector3(0, radius, 0), yColor, 0.3));
      this.add(axisLine(new Vector3(0, -radius, 0), yColor, 0.2));
    }

    this.add(axisLine(new Vector3(0, 0, radius), zColor));
    this.add(axisLine(new Vector3(0, 0, -radius), zColor));
  }
}
