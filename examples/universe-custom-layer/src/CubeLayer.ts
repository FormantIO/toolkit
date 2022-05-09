import { UniverseLayer, Label, Hand, Controller } from "@formant/universe";
import {
  BoxGeometry,
  MeshBasicMaterial,
  Mesh,
  Raycaster,
  WebXRManager,
} from "three";

export class CubeLayer extends UniverseLayer {
  static layerTypeId = "cube";
  static commonName = "Cube";
  static description = "This is just a simple cube.";

  geo = new BoxGeometry(1, 1, 1);
  mat = new MeshBasicMaterial({ color: 0x20a0ff });
  cube = new Mesh(this.geo, this.mat);

  label = new Label("controller");

  init() {
    this.add(this.cube);
  }

  onPointerMove(raycaster: Raycaster): void {
    let intersects = raycaster.intersectObject(this.cube).length > 0;
    this.mat.color.set(intersects ? 0x20a0ff : 0xffffff);
  }

  onPointerDown(raycaster: Raycaster): void {
    if (raycaster.intersectObject(this.cube).length > 0) {
      this.mat.color.set(0xff0000);
    }
  }

  onPointerUp(_raycaster: Raycaster, _button: number): void {
    this.showSnackbar("Clicked!");
  }

  onControllersMoved(controllers: Controller[]): void {
    let intersected = false;
    controllers.forEach((_) => {
      if (!intersected) {
        intersected = _.raycaster.intersectObject(this.cube).length > 0;
      }
    });

    this.mat.color.set(intersected ? 0x20a0ff : 0xffffff);
  }

  onControllerButtonChanged(
    controller: Controller,
    _button: number,
    value: number
  ): void {
    if (
      value === 1 &&
      controller.raycaster.intersectObject(this.cube).length > 0
    ) {
      controller.pulse(1, 100);
    }
  }

  onEnterVR(_xr: WebXRManager): void {
    this.mat.color.set(0x00ff00);
  }

  onExitVR(_xr: WebXRManager): void {
    this.mat.color.set(0xffffff);
  }

  onHandsMoved(hands: Hand[]): void {
    let intersects = false;
    let text = "";
    hands.forEach((hand: Hand, i) => {
      if (hand && hand.intersectBoxObject(this.cube)) {
        const p = hands[i].controller.joints["index-finger-tip"].position;
        text = p.x.toFixed(4) + "," + p.y.toFixed(4) + "," + p.z.toFixed(4);
        intersects = true;
      }
    });

    if (intersects) {
      this.label.text = text;
      this.mat.color.setHex(0xff0000);
    } else {
      this.label.text = "hands entered, touch the cube";
      this.mat.color.setHex(0x20a0ff);
    }
  }

  onHandsEnter(_hands: Hand[]): void {
    this.add(this.label);
  }

  onHandsLeave(_hands: Hand[]): void {
    this.add(this.label);
  }

  destroy(): void {
    this.geo.dispose();
    this.mat.dispose();
  }
}
