import { UniverseLayer, Hand, Label } from "@formant/universe";
import {
  BoxBufferGeometry,
  Mesh,
  MeshBasicMaterial,
  Object3D,
  TextureLoader,
} from "three";

export class HandsLayer extends UniverseLayer {
  static layerTypeId = "hands";

  static commonName = "Hands";

  static description = "This is a scene that shows hand poses";

  labelLeft = new Label("");

  labelRight = new Label("");

  base = new Object3D();

  cube = new Mesh(
    new BoxBufferGeometry(1 / 2, 0.66 / 2, 0),
    new MeshBasicMaterial({
      map: new TextureLoader().load(
        "https://i0.wp.com/ilikeinterfaces.com/wp-content/uploads/2016/04/00_15_4200053.png"
      ),
    })
  );

  init() {
    this.base.add(this.cube);
    this.add(this.base);
    this.cube.position.set(0, 0, 0.4);
  }

  onHandsEnter(hands: Hand[]): void {
    hands[0].add(this.labelLeft);
    const wrist = hands[0].getJoint("wrist");
    if (wrist) {
      this.labelLeft.position.set(
        wrist.position.x,
        wrist.position.y,
        wrist.position.z
      );
    }

    hands[1].add(this.labelRight);
    const wrist2 = hands[1].getJoint("wrist");
    if (wrist2) {
      this.labelRight.position.set(
        wrist2.position.x,
        wrist2.position.y,
        wrist2.position.z
      );
    }
  }

  onUpdate(_delta: number): void {
    if (this.camera) {
      const camera = this.camera();
      this.labelLeft.text = `${camera.position.x.toFixed(
        2
      )}, ${camera.position.y.toFixed(2)}, ${camera.position.z.toFixed(2)}`;
      this.base.position.set(
        camera.position.x,
        camera.position.y,
        camera.position.z
      );
    }
  }

  onHandsMoved(hands: Hand[]): void {
    const wrist = hands[0].getJoint("wrist");
    if (wrist) {
      this.labelLeft.position.set(
        wrist.position.x,
        wrist.position.y,
        wrist.position.z
      );
    }
    const wrist2 = hands[1].getJoint("wrist");
    if (wrist2) {
      this.labelRight.position.set(
        wrist2.position.x,
        wrist2.position.y,
        wrist2.position.z
      );
    }
  }
}
