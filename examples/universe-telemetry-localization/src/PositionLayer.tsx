import { defined, UniverseLayer, IOdometry } from "@formant/universe";
import * as uuid from "uuid";
import { Quaternion, MeshBasicMaterial, Mesh, CircleGeometry } from "three";

export class PositionLayer extends UniverseLayer {
  static layerTypeId = "PositionLayer";
  static commonName = "Localization layer";

  static description = "A test layer";

  positionUnsubsciber: undefined | (() => void);

  geo = new CircleGeometry(0.1, 16);
  mat = new MeshBasicMaterial({ color: 0x20a0ff });
  cube = new Mesh(this.geo, this.mat);

  init(): void {
    this.add(this.cube);
    this.positionUnsubsciber = defined(this.universeData).subscribeToOdometry(
      defined(this.getLayerContext()).deviceId,
      {
        id: uuid.v4(),
        sourceType: "telemetry",
        streamName: "map",
        streamType: "localization",
      },
      (d) => {
        if (typeof d === "symbol") {
          throw new Error("unhandled data status");
        }
        const odom = d as IOdometry;
        const pos = odom.pose.translation;
        const rot = odom.pose.rotation;
        this.position.set(pos.x, pos.y, pos.z);
        this.setRotationFromQuaternion(
          new Quaternion(rot.x, rot.y, rot.z, rot.w)
        );
      }
    );
  }
}
