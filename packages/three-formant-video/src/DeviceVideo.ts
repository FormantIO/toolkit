import * as THREE from "three";
import { Device, RealtimeVideoStream } from "@formant/data-sdk";
import { defined } from "../../common/defined";
import { H264BytestreamCanvasDrawer } from "@formant/ui-sdk-realtime-player-core";
// @ts-ignore
import RealtimePlayerWorker from "../node_modules/@formant/ui-sdk-realtime-player-core-worker/dist/ui-sdk-realtime-player-core-worker.umd?worker&inline";

export class DeviceVideo extends THREE.Object3D {
  videoStream: RealtimeVideoStream | undefined;
  drawer: H264BytestreamCanvasDrawer;
  canvas: HTMLCanvasElement;
  mesh: THREE.Mesh;
  constructor(private device: Device, videoStreamName?: string) {
    super();
    this.drawer = new H264BytestreamCanvasDrawer(
      () => new RealtimePlayerWorker(),
      () => {},
      () => {}
    );
    this.canvas = document.createElement("CANVAS") as HTMLCanvasElement;

    const texture = new THREE.CanvasTexture(this.canvas);

    const material = new THREE.MeshBasicMaterial({
      map: texture,
    });

    const geometry = new THREE.BoxGeometry(1, 1, 0);
    this.mesh = new THREE.Mesh(geometry, material);
    this.add(this.mesh);

    (async () => {
      const videoStreams = await this.device.getRealtimeVideoStreams();
      if (!videoStreamName) {
        if (videoStreams.length === 0) {
          throw new Error(`Device ${device?.name} has no video streams`);
        }
        this.videoStream = videoStreams[0];
      } else {
        this.videoStream = videoStreams.find((_) => _.name === videoStreamName);
        if (!this.videoStream) {
          throw new Error(
            `Device ${device?.name} has no video stream ${videoStreamName}`
          );
        }
      }
      await this._start();
    })();
  }
  async _start() {
    defined(this.device).addRealtimeListener((_peer, message) => {
      if (message.header.stream.streamName === this.videoStream?.name) {
        this.drawVideoFrame(message.payload.h264VideoFrame);
      }
    });
    await defined(this.device).startListeningToRealtimeVideo(
      defined(this.videoStream)
    );

    this.drawer.setCanvas(this.canvas);
    this.drawer.start();
  }

  drawVideoFrame(h264Frame: any) {
    this.drawer.receiveEncodedFrame(h264Frame);
    this.mesh.scale.set(1, this.canvas.height / this.canvas.width, 0);
    (this.mesh.material as any).map.needsUpdate = true;
    (this.mesh.material as any).needsUpdate = true;
  }
}
