import { Object3D, Sphere, Box3, Texture, Mesh, Vector3 } from "three";
import { XRHandMeshModel } from "../../three-utils/webxr/XRHandMeshModel";

const TOUCH_RADIUS = 0.01;
const POINTING_JOINT = "index-finger-tip";

export interface IButton {
  onPress(): void;
  onClear(): void;
  isPressed(): boolean;
  whilePressed(): void;
}

class FormantHandModel extends Object3D {
  controller: Object3D;

  motionController: XRHandMeshModel | null;

  envMap: Texture | null;

  mesh: Mesh | null;

  constructor(controller: Object3D) {
    super();

    this.controller = controller;
    this.motionController = null;
    this.envMap = null;

    this.mesh = null;

    controller.addEventListener("connected", (event) => {
      const xrInputSource = event.data;

      if (xrInputSource.hand && !this.motionController) {
        this.motionController = new XRHandMeshModel(
          this,
          controller,
          (this as any).path,
          xrInputSource.handedness
        );
      }
    });

    controller.addEventListener("disconnected", () => {
      this.clear();
      this.motionController = null;
    });
  }

  getPointerPosition(): Vector3 | null {
    const indexFingerTip = (this.motionController as any).joints[
      POINTING_JOINT
    ];
    if (indexFingerTip) {
      return indexFingerTip.position;
    }
    return null;
  }

  intersectBoxObject(boxObject: Object3D): boolean {
    const pointerPosition = this.getPointerPosition();
    if (pointerPosition) {
      const indexSphere = new Sphere(pointerPosition, TOUCH_RADIUS);
      const box = new Box3().setFromObject(boxObject);
      return indexSphere.intersectsBox(box);
    }
    return false;
  }

  checkButton(button: Object3D & IButton): void {
    if (this.intersectBoxObject(button)) {
      button.onPress();
    } else {
      button.onClear();
    }

    if (button.isPressed()) {
      button.whilePressed();
    }
  }
}

export { FormantHandModel };
