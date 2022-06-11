import { UniverseLayer } from "../src/main";

export class TestLayer extends UniverseLayer {
  static layerTypeId: string = "test";

  static commonName = "Test";

  static description = "Layer for testing.";

  init() {
    const c = this.createGltf(
      "https://formant-3d-models.s3.us-west-2.amazonaws.com/poser.glb"
    );
    c.position.y = 1;
    this.add(c);

    const r = this.createRoundedRectangle({
      width: 0.5,
      height: 0.25,
      radius: 0.05,
    });
    r.position.y = 0.5;
    this.add(r);

    const img = this.createImage("https://i.imgur.com/sZJWdbK.gif");
    img.position.y = 0.5;
    img.position.z = 0.0001;
    img.scale.set(0.25, 0.25, 0.25);
    this.add(img);

    const t = this.createText("Formant", { fontSize: 60 });
    t.position.y = 0.2;
    t.position.z = 0.0002;
    this.add(t);
  }
}
