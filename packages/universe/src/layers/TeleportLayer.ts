import * as THREE from "three";
import { Vector3, WebXRManager, XRReferenceSpace, XRSession } from "three";
import { Controller } from "../components/viewer/Controller";
import { Hand, HandPose } from "../components/viewer/Hand";
import { UniverseLayer } from "./UniverseLayer";

export class TeleportLayer extends UniverseLayer {
  static layerTypeId = "teleport";

  static commonName = "Teleport";

  static description = "This a plane on the floor responsive to teleportation.";

  xr?: WebXRManager;

  intersection?: THREE.Vector3;

  originalBaseReferenceSpace?: XRReferenceSpace;

  session: XRSession | null = null;

  lastLeftHandPose: HandPose = "unknown";

  lastLeftHandOpenPoseEndedTime: number = 0;

  teleporting: boolean = false;

  onControllersMoved(controllers: Controller[]): void {
    const { raycaster } = controllers[0];
    const intersects = raycaster.intersectObjects([this.getFloor()]);

    if (intersects.length > 0) {
      this.intersection = intersects[0].point;
    } else {
      this.intersection = undefined;
    }
  }

  onHandsMoved(hands: Hand[]): void {
    const { raycaster } = hands[0];
    const intersects = raycaster.intersectObjects([this.getFloor()]);

    if (intersects.length > 0) {
      this.intersection = intersects[0].point;
    } else {
      this.intersection = undefined;
    }
  }

  onControllerButtonChanged(
    controller: Controller,
    button: number,
    value: number
  ): void {
    if (
      controller.handedness === "left" &&
      button === 3 &&
      value === 1 &&
      this.xr &&
      this.intersection
    ) {
      this.teleportTo(this.intersection);
      controller.pulse(1, 100);
    }
  }

  onHandPosesChanged(hands: Hand[]): void {
    const pose = hands[0].getHandPose();
    if (this.lastLeftHandPose === "open") {
      this.lastLeftHandOpenPoseEndedTime = Date.now();
    }
    if (pose === "fist" && this.intersection) {
      const timeSinceLastOpenHandPose =
        Date.now() - this.lastLeftHandOpenPoseEndedTime;
      if (timeSinceLastOpenHandPose < 200) {
        this.teleportTo(this.intersection);
      }
    }
    this.lastLeftHandPose = pose;
  }

  teleportTo(p: Vector3) {
    const c = this.getCurrentCamera().position.distanceTo(p);
    if (c > 3) {
      return;
    }
    if (this.xr) {
      const session = this.xr.getSession();
      const baseReferenceSpace = this.xr.getReferenceSpace();
      if (baseReferenceSpace) {
        if (!this.originalBaseReferenceSpace || this.session !== session) {
          this.originalBaseReferenceSpace = baseReferenceSpace;
          this.session = session;
        }

        const offsetPosition = {
          x: -p.x,
          y: -p.y,
          z: -p.z,
          w: 1,
        };
        const offsetRotation = new THREE.Quaternion();
        // @ts-ignore
        // eslint-disable-next-line
        const transform = new XRRigidTransform(offsetPosition, offsetRotation);
        const teleportSpaceOffset =
          this.originalBaseReferenceSpace.getOffsetReferenceSpace(transform);
        // @ts-ignore
        this.xr.setReferenceSpace(teleportSpaceOffset);
      }
    }
  }

  onEnterVR(xr: WebXRManager): void {
    this.xr = xr;
  }

  onExitVR(_xr: THREE.WebXRManager): void {
    this.xr = undefined;
  }
}
