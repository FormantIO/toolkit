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

  marker = new THREE.Mesh(
    new THREE.CircleGeometry(0.25, 32),
    new THREE.MeshBasicMaterial({ color: 0xbac4e2 })
  );

  floor = new THREE.Mesh(
    new THREE.PlaneGeometry(100, 100, 2, 2),
    new THREE.MeshBasicMaterial({
      color: 0x808080,
      transparent: true,
      opacity: 0,
    })
  );

  intersection?: THREE.Vector3;

  originalBaseReferenceSpace?: XRReferenceSpace;

  session: XRSession | null = null;

  lastPose: HandPose = "unknown";

  init() {
    this.floor.rotation.set(-Math.PI / 2, 0, 0);
    this.marker.rotation.set(-Math.PI / 2, 0, 0);
    this.add(this.marker);
    this.add(this.floor);
  }

  onControllersMoved(controllers: Controller[]): void {
    const { raycaster } = controllers[0];
    const intersects = raycaster.intersectObjects([this.floor]);

    if (intersects.length > 0) {
      this.intersection = intersects[0].point;
      this.marker.position.set(
        this.intersection.x,
        this.intersection.y,
        this.intersection.z
      );
    }
  }

  onHandsMoved(hands: Hand[]): void {
    const { raycaster } = hands[0];
    const intersects = raycaster.intersectObjects([this.floor]);

    if (intersects.length > 0) {
      this.intersection = intersects[0].point;
      this.marker.position.set(
        this.intersection.x,
        this.intersection.y,
        this.intersection.z
      );
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
    if (this.lastPose === "open" && pose === "fist" && this.intersection) {
      this.teleportTo(this.intersection);
    }
  }

  teleportTo(p: Vector3) {
    if (this.xr) {
      const session = this.xr.getSession();
      const baseReferenceSpace = this.xr.getReferenceSpace();
      if (baseReferenceSpace && this.camera) {
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
