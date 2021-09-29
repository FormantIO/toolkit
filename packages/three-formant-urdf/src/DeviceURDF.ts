import * as THREE from "three";
import { Device, RealtimeJointStateStream } from "@formant/data-sdk";
import { defined } from "../../common/defined";
import { LoadingManager } from "three";
import URDFLoader from "urdf-loader";
import { Object3D } from "three";

export class DeviceURDF extends THREE.Object3D {
  jointStateStream: RealtimeJointStateStream | undefined;
  constructor(private device: Device, jointStateStreamName?: string) {
    super();
    const manager = new LoadingManager();
    const loader = new URDFLoader(manager);
    loader.load(
      "https://cdn.jsdelivr.net/gh/gkjohnson/urdf-loaders@0.10.2/urdf/T12/urdf/T12.URDF",
      (robot) => {
        this.add(robot as unknown as Object3D);
      }
    );
    (async () => {
      const joinStateStreams = await this.device.getRealtimeVideoStreams();
      if (!jointStateStreamName) {
        if (joinStateStreams.length === 0) {
          throw new Error(`Device ${device?.name} has no joint state streams`);
        }
        this.jointStateStream = joinStateStreams[0];
      } else {
        this.jointStateStream = joinStateStreams.find(
          (_) => _.name === jointStateStreamName
        );
        if (!this.jointStateStream) {
          throw new Error(
            `Device ${device?.name} has no joint state stream ${jointStateStreamName}`
          );
        }
      }
      await this._start();
    })();
  }

  async _start() {
    defined(this.device).addRealtimeListener((_peer, message) => {
      if (message.header.stream.streamName === this.jointStateStream?.name) {
      }
    });
    await defined(this.device).startListeningToRealtimeJointState(
      defined(this.jointStateStream)
    );
  }
}
