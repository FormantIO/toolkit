import * as THREE from "three";
import { WebXRManager, XRReferenceSpace } from "three";
import { Controller } from "../components/viewer/Controller";
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

  init() {
    this.add(this.marker);
    this.add(this.floor);
  }

  onControllersMoved(controllers: Controller[]): void {
    const { raycaster } = controllers[0];
    const intersects = raycaster.intersectObjects([this.floor]);

    if (intersects.length > 0) {
      this.intersection = intersects[0].point;
      console.log(this.intersection.x, -this.intersection.z);
      this.marker.position.set(this.intersection.x, -this.intersection.z, 0);
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
      const baseReferenceSpace = this.xr.getReferenceSpace();
      if (baseReferenceSpace && this.camera) {
        if (!this.originalBaseReferenceSpace) {
          this.originalBaseReferenceSpace = baseReferenceSpace;
        }

        const offsetPosition = {
          x: -this.intersection.x,
          y: -this.intersection.y,
          z: -this.intersection.z,
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
        controller.pulse(1, 100);
      }
    }
  }

  onEnterVR(xr: WebXRManager): void {
    this.xr = xr;
  }
}
