import {
  BackSide,
  BoxGeometry,
  CanvasTexture,
  Mesh,
  MeshBasicMaterial,
  SphereGeometry,
} from "three";
import * as uuid from "uuid";
import { H264BytestreamCanvasDrawer } from "@formant/ui-sdk-realtime-player-core";
import { IUniverseData } from "../model/IUniverseData";
import { UniverseLayer } from "./UniverseLayer";
import { defined } from "../../../common/defined";
import { IH264VideoFrame } from "../../../data-sdk/src/model/IH264VideoFrame";
import { LayerSuggestion } from "./LayerRegistry";
// @ts-ignore
// eslint-disable-next-line import/no-unresolved
import RealtimePlayerWorker from "../../node_modules/@formant/ui-sdk-realtime-player-core-worker/dist/ui-sdk-realtime-player-core-worker.umd?worker&inline";

export class RealtimeVideoLayer extends UniverseLayer {
  static layerTypeId = "rtc_video";

  static commonName = "Realtime Video";

  static description = "A video representing a realtime camera.";

  static usesData = true;

  static fields = {
    video_shape: {
      name: "Shape",
      description: "The shape you'd like the video to be",
      placeholder: "sphere",
      value: "",
      type: "text",
      location: ["create"],
    },
  };

  static async getLayerSuggestions(
    universeData: IUniverseData,
    deviceContext?: string
  ): Promise<LayerSuggestion[]> {
    const dataLayers: LayerSuggestion[] = [];
    if (deviceContext) {
      (await universeData.getHardwareStreams(deviceContext)).forEach(
        (stream) => {
          if (stream.rtcStreamType === "h264-video-frame") {
            dataLayers.push({
              sources: [
                {
                  id: uuid.v4(),
                  sourceType: "hardware",
                  rtcStreamName: stream.name,
                },
              ],
              layerType: RealtimeVideoLayer.layerTypeId,
            });
          }
        }
      );
    }
    return dataLayers;
  }

  loaded: boolean = false;

  drawer?: H264BytestreamCanvasDrawer;

  mesh?: Mesh;

  init() {
    const dataSource = defined(this.layerDataSources)[0];
    this.drawer = new H264BytestreamCanvasDrawer(
      () => new RealtimePlayerWorker(),
      () => {},
      () => {}
    );
    defined(this.universeData).subscribeToRealtimeVideo(
      defined(this.layerContext),
      defined(dataSource),
      this.onData
    );
    const canvas = document.createElement("CANVAS") as HTMLCanvasElement;
    canvas.width = 0;
    canvas.height = 0;
    this.drawer.setCanvas(canvas);
    this.drawer.start();
  }

  onData = (frame: IH264VideoFrame) => {
    const drawer = defined(this.drawer);
    drawer.receiveEncodedFrame(frame);
    // we lazily create this canvas because threejs doesn't like size changes
    const canvas = defined(drawer.canvas);
    if (!this.mesh && canvas.width > 0 && canvas.height > 0) {
      const texture = new CanvasTexture(canvas);

      const shapeField = (this.layerFields || {}).video_shape;
      const shape = shapeField.value;
      if (shape === "sphere") {
        const material = new MeshBasicMaterial({
          map: texture,
          side: BackSide,
        });
        const geometry = new SphereGeometry(0.3);
        this.mesh = new Mesh(geometry, material);
      } else {
        const material = new MeshBasicMaterial({
          map: texture,
        });
        const geometry = new BoxGeometry(1, 1, 0);
        const ninetyDegrees = Math.PI / 2;
        this.mesh = new Mesh(geometry, material);
        this.mesh.rotation.set(ninetyDegrees, 0, 0);
      }
      this.add(this.mesh);
      if (shape !== "sphere") {
        this.mesh.scale.set(1, canvas.height / canvas.width, 0);
      }
      (this.mesh.material as any).map.needsUpdate = true;
      (this.mesh.material as any).needsUpdate = true;
    }
  };
}
