import { Object3D } from "three";
import { UniverseLayer } from "../src/main";

export class TestLayer extends UniverseLayer {
  static layerTypeId: string = "test";

  static commonName = "Test";

  static description = "Layer for testing.";

  createButton(name: string) {
    const o = new Object3D();
    const r = this.createRoundedRectangle({
      width: 0.35,
      height: 0.15,
      radius: 0.05,
      color: 0x2d3855,
    });
    o.add(r);

    const t = this.createText(name, { fontSize: 16 });
    t.position.x = 0.0002;
    o.add(t);
    return o;
  }

  init() {
    const title = this.createText("AR3-EC2", { fontSize: 40 });
    title.position.z = 0.8;
    title.position.y = 1;
    this.add(title);
    const subtitle = this.createText("Powered by PickNik", { fontSize: 12 });
    subtitle.position.z = 0.7;
    subtitle.position.y = 1;
    subtitle.position.x = 0.0002;
    this.add(subtitle);
    const b1 = this.createButton("Reset");
    const b2 = this.createButton("Execute");
    b1.position.z = 0.5;
    b2.position.z = 0.5;

    b1.position.y = 1 + 0.19;
    b2.position.y = 1 + -0.19;
    this.add(b1);
    this.add(b2);
  }
}
