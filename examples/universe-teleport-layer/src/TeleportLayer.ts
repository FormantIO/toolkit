import { UniverseLayer, Controller } from "@formant/universe";
import * as THREE from "three";
import { Raycaster, WebXRManager } from "three";

export class TeleportLayer extends UniverseLayer {
  static layerTypeId = "teleport";
  static commonName = "Teleport";
  static description = "This a plane on the floor responsive to teleportation.";

  xr?: WebXRManager;

  marker = new THREE.Mesh(
    new THREE.CircleGeometry(0.25, 32),
    new THREE.MeshBasicMaterial({ color: 0x808080 })
  );

  floor = new THREE.Mesh(
    new THREE.PlaneGeometry(4.8, 4.8, 2, 2),
    new THREE.MeshBasicMaterial({
      color: 0x808080,
      transparent: true,
      opacity: 0.25,
    })
  );

  intersection: THREE.Vector3 = new THREE.Vector3();

  init() {
    this.add(this.marker);
    this.add(this.floor);
  }

  onControllersMoved(
    _controllers: Controller[],
    raycasters: Raycaster[]
  ): void {
    const raycaster = raycasters[0];
    const intersects = raycaster.intersectObjects([this.floor]);

    if (intersects.length > 0) {
      console.log("intersecting");
      this.intersection = intersects[0].point;
      console.log(this.intersection);
      this.marker.position.set(
        this.intersection.x,
        this.intersection.y,
        this.intersection.z
      );
    }
  }

  onControllerButtonChanged(
    controller: Controller,
    raycaster: Raycaster,
    button: number,
    value: number
  ): void {
    console.log(button);
  }

  onEnterVR(xr: WebXRManager): void {
    this.xr = xr;
  }
}
