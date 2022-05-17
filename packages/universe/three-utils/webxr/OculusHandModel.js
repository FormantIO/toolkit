import { Object3D, Sphere, Box3 } from "three";
import { XRHandMeshModel } from "./XRHandMeshModel.js";

const TOUCH_RADIUS = 0.01;
const POINTING_JOINT = "index-finger-tip";

class OculusHandModel extends Object3D {
  constructor(controller) {
    super();

    this.controller = controller;
    this.motionController = null;
    this.envMap = null;

    this.mesh = null;

    controller.addEventListener("connected", (event) => {
      const xrInputSource = event.data;

      if (xrInputSource.hand && !this.motionController) {
        this.xrInputSource = xrInputSource;

        this.motionController = new XRHandMeshModel(
          this,
          controller,
          this.path,
          xrInputSource.handedness
        );
      }
    });

    controller.addEventListener("disconnected", () => {
      this.clear();
      this.motionController = null;
    });
  }

  updateMatrixWorld(force) {
    super.updateMatrixWorld(force);

    if (this.motionController) {
      this.motionController.updateMesh();
    }
  }

  getPointerPosition() {
    const indexFingerTip = this.controller.joints[POINTING_JOINT];
    if (indexFingerTip) {
      if (
        indexFingerTip.position.x !== 0 &&
        indexFingerTip.position.y !== 0 &&
        indexFingerTip.position.z !== 0
      ) {
        return indexFingerTip.position;
      }
    }
    return null;
  }

  getJoint(joint) {
    return this.controller.joints[joint];
    if (j) {
      return j;
    }
    return null;
  }

  getHandPose() {
    let pose = "unknown";
    if (this.controller) {
      const thumb = this.getJoint("thumb-tip");
      const finger1 = this.getJoint("index-finger-tip");
      const palm1 = this.getJoint("index-finger-metacarpal");
      let distance1 = null;
      if (finger1 && palm1)
        distance1 = finger1.position.distanceTo(palm1.position);
      let pinchDistance = null;
      if (thumb && finger1)
        pinchDistance = thumb.position.distanceTo(finger1.position);
      const finger2 = this.getJoint("middle-finger-tip");
      const palm2 = this.getJoint("middle-finger-metacarpal");
      let distance2 = null;
      if (finger2 && palm2)
        distance2 = finger2.position.distanceTo(palm2.position);
      const finger3 = this.getJoint("ring-finger-tip");
      const palm3 = this.getJoint("ring-finger-metacarpal");
      let distance3 = null;
      if (finger3 && palm3)
        distance3 = finger3.position.distanceTo(palm3.position);
      const finger4 = this.getJoint("pinky-finger-tip");
      const palm4 = this.getJoint("pinky-finger-metacarpal");
      let distance4 = null;
      if (finger4 && palm4)
        distance4 = finger4.position.distanceTo(palm4.position);
      if (this.controller.visible) {
        if (pinchDistance && pinchDistance < 0.02) {
          pose = "pinch";
        } else if (
          distance1 &&
          distance1 >= 0.065 &&
          distance2 &&
          distance2 >= 0.065 &&
          distance3 &&
          distance3 >= 0.065 &&
          distance4 &&
          distance4 >= 0.065
        ) {
          pose = "open";
        } else if (
          distance1 &&
          distance1 < 0.065 &&
          distance2 &&
          distance2 < 0.065 &&
          distance3 &&
          distance3 < 0.065 &&
          distance4 &&
          distance4 < 0.065
        ) {
          pose = "fist";
        } else if (distance1 && distance2 && distance3 && distance4) {
          pose = "point";
        }
      }
    }

    return pose;
  }

  intersectBoxObject(boxObject) {
    const pointerPosition = this.getPointerPosition();
    if (pointerPosition) {
      const indexSphere = new Sphere(pointerPosition, TOUCH_RADIUS);
      const box = new Box3().setFromObject(boxObject);
      return indexSphere.intersectsBox(box);
    } else {
      return false;
    }
  }

  checkButton(button) {
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

export { OculusHandModel };
